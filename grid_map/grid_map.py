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

def bresenham_line(x1, y1, x2, y2):
    points = []
    
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    
    err = dx - dy
    
    while True:
        points.append((x1, y1))
        
        if x1 == x2 and y1 == y2:
            break
        
        e2 = 2 * err
        
        if e2 > -dy:
            err -= dy
            x1 += sx
        
        if e2 < dx:
            err += dx
            y1 += sy
    
    return points


json_data = open('./map_sonar_round.json') #./grid_map/map_boxes_0.json
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

scans_in_global_x = []
scans_in_global_y = []

for data_scan, data_pose in zip(scan_arr, pose_arr):
    # plt.scatter(data_pose[0], data_pose[1], color = "red")
    delta_x = data_pose[0] - ref_x
    delta_y = data_pose[1] - ref_y
    delta_theta = data_pose[2] - ref_theta
    for x,y in data_scan:
        rotated_x = x * math.cos(math.radians(delta_theta)) - y * math.sin(math.radians(delta_theta))
        rotated_y = x * math.sin(math.radians(delta_theta)) + y * math.cos(math.radians(delta_theta))
        x = rotated_x + delta_x
        y = rotated_y + delta_y

        scans_in_global_x.append(x)
        scans_in_global_y.append(y)
#         plt.scatter(x,y, color="black")
# plt.show()

map_width = 120
map_length = 120
zero_point_x = int(map_width/2)
zero_point_y = int(map_length/2)
grid_map = [[50] * map_width for _ in range(map_length)]


for i, j in zip(scans_in_global_x, scans_in_global_y):
    x_coord = int(i)*10 + zero_point_x
    y_coord = int(j)*10 + zero_point_y
    line_points = bresenham_line(zero_point_x, zero_point_y, x_coord, y_coord)
    for point in line_points:
        grid_map[point[0]][point[1]] = 5
    
    grid_map[x_coord][y_coord] = 95
x, y = np.where(np.array(grid_map) == 50)
plt.scatter(x, y, color="grey")

x, y = np.where(np.array(grid_map) == 5)
plt.scatter(x, y, color="white")

x, y = np.where(np.array(grid_map) == 95)
plt.scatter(x, y, color="black")




plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.show()
