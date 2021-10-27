import random 
import math 
import matplotlib.pyplot as plt
from matplotlib.patches import  Polygon
from matplotlib.collections import PatchCollection
import numpy as np


class RRT:
    class Node:
        def __init__(self, x, y , parent = None) :
            self.x = x
            self.y = y
            self.parent = parent

               
    def __init__(self, start, goal, step_length, goal_sample_rate, iter_max, obstacle_list, boundary ):
        # self.start = self.Node(start[0], start[1])
        # self.goal = self.Node(goal[0], goal[1])
        
        self.start = start
        self.goal = goal
        
        self.step_length = step_length
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.obstacle_list= obstacle_list
        self.boundary = boundary  #[[x0, y0], [x1, y1]]
        self.bx0 = boundary[0][0]
        self.bx1 = boundary[1][0]
        self.by0 = boundary[0][1]
        self.by1 = boundary[1][1]

    def get_random_node(self):
        print("random node")
        if random(0, 100) >= self.goal_sample_rate:
            randomnode = self.Node(random(self.bx0, self.bx1), random(self.by0, self.by1))
        else:
            randomnode = self.goal
        return randomnode
    def get_orientation(self, p1, p2, p3):
        theta12 = ((p2[1] - p1[1]) / (p2[0] - p1[0]))  #p1[0] is x of point1 , p1[1] is y of point1
        theta23 = ((p3[1] - p2[1]) / (p3[0] - p2[0])) 
        
        print()
    
    def check_node_collision(self, node):
        collision = False
        print("check node collision")
        for obstacle in self.obstacle_list:
            for i in range(len(obstacle)):
                print("point", obstacle[i] , " ", obstacle[i-1])
        print("my node ", node.x, " ", node.y )
        
    
    def planning(node_list ):
        print("planning ")
        
    def plotting(self, node_list, node_path):
            obstacle_list = self.obstacle_list
            # boundary = self.boundary
            
            fig, ax = plt.subplots()
            #plot obstacle list 
            patches = []
            for obstacle in obstacle_list:
                polygon = Polygon(obstacle, True)
                patches.append(polygon)
            patch_collection = PatchCollection(patches, alpha=0.4)
            patch_collection.set_alpha(0.8)
            patch_collection.set_color("k")
            ax.add_collection(patch_collection)
            
            #plot start,goal
            plt.plot(self.start.x, self.start.y, "ob")
            plt.plot(self.goal.y,  self.goal.y, "xr") 
            
            xbound = [self.bx0,self.bx0,self.bx1,self.bx1,self.bx0]
            ybound = [self.by0,self.by1, self.by1,self.by0, self.by0]
            plt.plot(xbound, ybound)
            
            plt.autoscale(True)
            plt.show()
        


def main():
    print("main")
    

    # start_node = [1,1]   # declare start position 
    # goal_node = [26, 29] # declare goal position
    
    # test_node = RRT.Node(1, 1)
    start_node =RRT.Node(1, 1)
    goal_node = RRT.Node(26, 29)
    
    #declare obstacle in map 
    obstacle1 = [[4,5], [6,3], [5,6]]
    obstacle2 = [[4,4], [2,4], [3,7]]
    obstacle_list =[obstacle1, obstacle2 ]
    
    #declare boundary
    boundary = [[0,0], [30,30]]

    node_list =[]
    node_path=[]
    RRTsolver = RRT(start_node, goal_node, 2, 2, 100,obstacle_list, boundary )
    RRTsolver.check_node_collision(start_node)
    RRTsolver.plotting(node_list, node_path)
    

            
if __name__ == '__main__':
    main()        
