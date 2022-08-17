import math
from time import time
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d
show_animation = True
from matplotlib.pyplot import figure

# figure(figsize=(18, 16), dpi=80)
class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, time_collision_gap):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.time_collision_gap = time_collision_gap
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy, event_lists, speed):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        print('start_node x ', start_node.x, ' start_node y ', start_node.y)
        open_set[self.calc_grid_index(start_node)] = start_node
        print("start node calc grid index : ", self.calc_grid_index(start_node))

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set,key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), ".c")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
               
                if len(closed_set.keys()) % 30 == 0:
                    plt.pause(0.000001) 
                    # print("=====")

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                # print('cost ', node.cost)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue
                
                # if collision in time stamp , skip 
                if self.verify_othernode_collision(event_lists, n_id, node.cost/speed):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry, event_list, pos_time = self.calc_final_path(goal_node, closed_set, speed)

        return rx, ry, event_list, pos_time

    def calc_final_path(self, goal_node, closed_set, speed):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        grid_id_list = []
        grid_id_list.append(self.calc_grid_index(goal_node))
        time_checkpoint = []
        time_checkpoint.append(round(closed_set[parent_index].cost/speed,3))
        while parent_index != -1:
            n = closed_set[parent_index]
            # print('cost of node ', n.cost)
            time_checkpoint.append(round(n.cost/speed, 3))
            grid_id_list.append(parent_index)
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        rx.reverse()
        ry.reverse()
        grid_id_list.reverse()
        time_checkpoint.reverse()
        event_list = [rx, ry , grid_id_list, time_checkpoint]
        pos_time = [rx, ry , time_checkpoint]
        return rx, ry , event_list, pos_time

    def pos_distribute_time_res(self, pos_time, time_res):
        time_step = 0.0
        time_steps = [[],[],[]]
        while time_step < pos_time[2][-1]:
            for i in range(0, len(pos_time[2])):
                if pos_time[2][i-1] < time_step and time_step < pos_time[2][i]:

                    ratio = (time_step-pos_time[2][i-1])/(pos_time[2][i]-pos_time[2][i-1])

                    rx_rat = pos_time[0][i-1] + (pos_time[0][i]-pos_time[0][i-1])*ratio
                    ry_rat = pos_time[1][i-1] + (pos_time[1][i]-pos_time[1][i-1])*ratio
                    time_steps[0].append(round(time_step, 2))
                    time_steps[1].append(round(rx_rat,3))
                    time_steps[2].append(round(ry_rat,3))

            time_step += time_res   
        time_steps[0].append(round(pos_time[2][-1], 3))
        time_steps[1].append(pos_time[0][-1])
        time_steps[2].append(pos_time[1][-1]) 
        return time_steps

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):

        # return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)
        return (node.y-1) * self.x_width + node.x 

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def verify_othernode_collision(self, event_lists, grid_id, time_stamp):
        if event_lists ==[]:
            # print("most prioritized")
            return False
        else:
            # print('number of event : ', len(event_lists))
            for single_event_index in range(0, len(event_lists)):

                # print('checking event: ', single_event_index)
                #single_event = event_list
                single_event = event_lists[single_event_index] 
                for i in range(0, len(single_event[0])- 1 ):
                    if single_event[2][-1] == grid_id:
                        # print('collsion to others node goal at ', grid_id)
                        return True
                    # print('single event  number of param ', len(single_event))
                    elif single_event[2][i] == grid_id:

                        # print('single event [2] ', single_event[2][i], ' and grid_id ', grid_id)
                        # print("got intersectin grid pos, checking time stamp")

                        if abs(single_event[3][i] - time_stamp) <= self.time_collision_gap :
                            # print(abs(single_event[3][i] - time_stamp))
                            # print("collision in time stamp")
                            # print('iter :', i, 'event ', single_event_index , ' timestamp',  single_event[3][i], 'grid-id ', single_event[2][i])
                            return True #means collision
            # else:
            # print(" collision free at ", grid_id)
            return False



    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 8.0  # [m]
    sy = 4.0  # [m]
    gx = 50.0  # [m]
    gy = 30.0  # [m]

    sx2 = 8.0  # [m]
    sy2 = 8.0  # [m]
    gx2 = 50.0  # [m]
    gy2 = 22.0  # [m]

    sx3 = 8.0  # [m]
    sy3 = 12.0  # [m]
    gx3 = 56.0  # [m]
    gy3 = 36.0  # [m]

    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]
    time_collision_gap = 1.0

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 50):
        ox.append(40.0)
        oy.append(60.0 - i)
    for i in range(8, 21):
        ox.append(i)
        oy.append(40)   
    for i in range(26, 40):
        ox.append(i)
        oy.append(11)   

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "^r")
        plt.plot(sx2, sy2, "ob")
        plt.plot(gx2, gy2, "^b")
        plt.plot(sx3, sy3, "ok")
        plt.plot(gx3, gy3, "^k")
        plt.grid(True)
        plt.axis("equal")

    a_star1 = AStarPlanner(ox, oy, grid_size, robot_radius,time_collision_gap)
    a_star2 = AStarPlanner(ox, oy, grid_size, robot_radius,time_collision_gap)
    a_star3 = AStarPlanner(ox, oy, grid_size, robot_radius,time_collision_gap)
    events = []
    rx, ry, event_list, pos_time1 = a_star1.planning(sx, sy, gx, gy, [], 2.5)
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
    print('len of event_list0 ', len(event_list[0]))
    print('len of event_list1 ', len(event_list[1]))
    print('len of event_list2 ', len(event_list[2]))
    print('len of event_list3 ', len(event_list[3]))
  
    events.append(event_list)
    rx2, ry2, event_list2, pos_time2 = a_star2.planning(sx2, sy2, gx2, gy2, events ,3)
    if show_animation:  
        plt.plot(rx2, ry2, "-b")
        plt.pause(0.001) 
    events.append(event_list2)

    rx3, ry3, event_list3, pos_time3 = a_star3.planning(sx3, sy3, gx3, gy3, events ,3)
    if show_animation:  
        plt.plot(rx3, ry3, "-k")
        plt.pause(0.001)       
    plt.show()
    plt.cla()
    figure(figsize=(18, 16), dpi=80)

    animate_res = 0.1 
    timesteps1 = a_star1.pos_distribute_time_res(pos_time1, animate_res)
    timesteps2 = a_star2.pos_distribute_time_res(pos_time2, animate_res)
    timesteps3 = a_star3.pos_distribute_time_res(pos_time3, animate_res)
    len1 = len(timesteps1[0])
    len2 = len(timesteps2[0])
    len3 = len(timesteps3[0])
    maxlen = max(len1, max(len2, len3))
    for i in range(0, maxlen):
        plt.cla()

        plt.axis("equal")
        

        plt.plot(rx, ry, "-r")
        plt.plot(rx2, ry2, "-b")
        plt.plot(rx3, ry3, "-k")
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "or")
        plt.plot(gx, gy, "^r")
        plt.plot(sx2, sy2, "ob")
        plt.plot(gx2, gy2, "^b")
        plt.plot(sx3, sy3, "ok")
        plt.plot(gx3, gy3, "^k")
        if i < len(timesteps1[0]):
            plt.plot(timesteps1[1][i], timesteps1[2][i], "or")
        elif i >= len(timesteps1[0]):
            plt.plot(timesteps1[1][-1], timesteps1[2][-1], "or")

        if i < len(timesteps2[0]):
            plt.plot(timesteps2[1][i], timesteps2[2][i], "ob")
        elif i >= len(timesteps2[0]):
            plt.plot(timesteps2[1][-1], timesteps2[2][-1], "ob")

        if i < len(timesteps3[0]):
            plt.plot(timesteps3[1][i], timesteps3[2][i], "oy")
        elif i >= len(timesteps3[0]):
            plt.plot(timesteps3[1][-1], timesteps3[2][-1], "oy")

        plt.pause(0.01)       
        
    plt.show()

        # print(timesteps3[0][i], end=" ")
        # print(timesteps3[1][i], end=" ")
        # print(timesteps3[2][i])
    # print(pos_time3[2][-1])
    # plt.show()


if __name__ == '__main__':
    main()
