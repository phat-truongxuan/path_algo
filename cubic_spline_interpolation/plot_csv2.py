import matplotlib.pyplot as plt
import numpy as np

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

