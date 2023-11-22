import random
import numpy as np
import json
import math
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from itertools import cycle


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x,y)


json_data = open('./grid_map/map_sonar_round.json') #./grid_map/map_boxes_0.json
data_loaded = json.load(json_data)

pose_arr = []
scan_arr = []

for data_sample in data_loaded:
    pose_arr.append(data_sample["pose"])
    scan_arr.append(data_sample["scan"])

i = 0
for data in scan_arr:
    xi = np.arange(0,512)
    theta = (np.pi/512 )*(xi-256)  # angle in radians
    [x,y] = pol2cart(data, theta)
    data_xy = [(x, y) for x, y in zip(x, y)]
    data_xy = [(x, y) for x, y in data_xy if all(math.isfinite(val) for val in (x, y))]
    scan_arr[i] = data_xy 
    i=i+1

ref_x = pose_arr[0][0] + 0.18
ref_y = pose_arr[0][1]
ref_theta = pose_arr[0][2]

plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')

scans_in_global_x = []
scans_in_global_y = []
min_x = 0
min_y = 0

for data_scan, data_pose in zip(scan_arr, pose_arr):
    plt.scatter(data_pose[0], data_pose[1], color = "red")
    delta_x = data_pose[0] - ref_x
    delta_y = data_pose[1] - ref_y
    delta_theta = data_pose[2] - ref_theta
    for x,y in data_scan:
        rotated_x = x * math.cos(math.radians(delta_theta)) - y * math.sin(math.radians(delta_theta))
        rotated_y = x * math.sin(math.radians(delta_theta)) + y * math.cos(math.radians(delta_theta))
        x = rotated_x + delta_x
        y = rotated_y + delta_y
        if x < min_x:
            min_x = x
        if y < min_y:
            min_y = y 
        scans_in_global_x.append(x)
        scans_in_global_y.append(y)
        #plt.scatter(x,y, color="black")
#plt.show()

max_x = 0
max_y = 0
for i in range(len(scans_in_global_x)):
    scans_in_global_x[i] = scans_in_global_x[i] + abs(min_x)
    if scans_in_global_x[i] > max_x:
        max_x = scans_in_global_x[i]

for i in range(len(scans_in_global_y)):
    scans_in_global_y[i] = scans_in_global_y[i] + abs(min_y)
    if scans_in_global_y[i] > max_y:
        max_y = scans_in_global_y[i]

grid_map = [[0] * 200] * 200
for data_x in scans_in_global_x:
    i = int(data_x * 10)
    for data_y in scans_in_global_x:
        j = int(data_y * 10)
        grid_map[i][j] = 1

for i in range(200):
    for j in range(200):
        if grid_map[i][j] == 1:
            plt.scatter(i,j, color="green")
        else:
            plt.scatter(i,j, color="grey")

plt.show()