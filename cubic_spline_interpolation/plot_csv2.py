import matplotlib.pyplot as plt
import numpy as np

# Fixing random state for reproducibility
np.random.seed(19680801)

import csv

x_in = []
y_in = []

with open('cubic_spline_interpolation/inputcubic.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        x_in.append(float(row[0]))
        y_in.append(float(row[1]))


x_ = []
y_ = []

with open('cubic_spline_interpolation/cubic.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        x_.append(float(row[0]))
        y_.append(float(row[1]))
        

print("so x ", x_)
print("so y ", y_)

plt.subplots(1)
plt.plot(x_in, y_in, "xb", label="input")
plt.plot(x_, y_, "-r", label="spline")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()
plt.show()


# plt.rcdefaults()
# fig, ax = plt.subplots()

# y_pos = ['pick blister', 'check_blister', 'foil_peeling', 'lens_pick', 'bottle_capping']

# performance = [10, 10 ,30, 40, 30]


# ax.bar(sothutu, sotunhien,  align='center')
# ax.set_xticks(sothutu)
# ax.set_xticklabels(sothutu)

# ax.set_xlabel('Performance')

# plt.show()