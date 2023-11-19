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

def fit_line(data_xy):
    index_1 = random.randint(0, len(data_xy) - 1)
    x1 = data_xy[index_1][0]
    y1 = data_xy[index_1][1]
    index_2 = random.randint(0, len(data_xy) - 1)
    x2 = data_xy[index_2][0]
    y2 = data_xy[index_2][1]
    while index_1 == index_2 or x2 == x1 :
        index_2 = random.randint(0, len(data_xy) - 1)
        x2 = data_xy[index_2][0]
        y2 = data_xy[index_2][1]

    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return a, b #return line equation

def compute_distance(point, line):
    a, b = line
    x, y = point 
    distance = abs(a * x - y + b) / np.sqrt(a**2 + 1)
    return distance

def ransac(data_xy, max_iterations, threshold, min_inliers):
    best_line = None
    best_inliers = []

    for _ in range(max_iterations):
        # Fit a line to a random sample of 2 points
        line = fit_line(data_xy)
        
        inliers = []
        for point in data_xy:
            distance = compute_distance(point, line)
            if distance < threshold:
                inliers.append(point)
        
        if len(inliers) > min_inliers and len(inliers) > len(best_inliers):
            #line = fit_line(inliers) # Approximating new line base of the inliners
            best_line = line
            best_inliers = inliers
        
        # Remove the inliers from the point set
        data_xy = [point for point in data_xy if point not in inliers]
        if len(data_xy) < min_inliers: break 
    return best_line, best_inliers


json_data = open('line_localization_1.json')
data = json.load(json_data)

xi = np.arange(0,512)
theta = (np.pi/512 )*(xi-256)  # angle in radians

scan_data = data[0]["scan"]
#scan_data = [x for x in scan_data if x <= 2]
replace = float('inf')
scan_data = [ replace if ele > 2 else ele for ele in scan_data]

[x,y] = pol2cart(scan_data, theta)
data_xy = [(x, y) for x, y in zip(x, y)]
data_xy = [(x, y) for x, y in data_xy if all(math.isfinite(val) for val in (x, y))]

max_iterations = 100
threshold = 0.01
min_inliers = 100
wall_point_array_x = []
wall_point_array_y = []
walls_x = []
walls_y = []
num_of_walls = 0
it=0
two_axis = []

while len(data_xy) >= min_inliers:
    if it>100:
        break

    best_line, inliers = ransac(data_xy, max_iterations, threshold, min_inliers)
    
    if best_line != None:
        inlier_x = [point[0] for point in inliers]
        inlier_y = [point[1] for point in inliers]
        wall_point_array_x.append(inlier_x)
        wall_point_array_y.append(inlier_y)
        data_xy = [point for point in data_xy if point not in inliers]
        a, b = best_line
        two_axis.append([a,b])
        wall_x = inlier_x
        wall_y = [a * idx + b for idx in wall_x]
        walls_x.append(wall_x)
        walls_y.append(wall_y)
        num_of_walls = num_of_walls + 1
    else:
        it = it + 1
# plt.scatter(x, y, label='All Points', color='gray', marker='o')

cmap = get_cmap('tab10')
colors = cycle(cmap.colors)

i = 0
while i < len(wall_point_array_x):
    color = next(colors)
    # plt.scatter(wall_point_array_x[i], wall_point_array_y[i], label="Inliers of wall " + str(i), color=color, marker='o')
    plt.plot(walls_x[i], walls_y[i], label="Best estimated Wall " + str(i), color=color)
    i = i+1

a1,b1 = two_axis[0]
a2,b2 = two_axis[1]
intersection_point=[(b2-b1) / (a1-a2), a1*((b2-b1) / (a1-a2)) + b1]

plt.plot(intersection_point[0],intersection_point[1], label='Intersection Point', color='gray', marker='o')
plt.plot(0, 0, label='Robot Cooridnates', color='Black', marker='o')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.legend()
plt.show()


x_coords = abs(a1 * 0 - 0 + b1) / np.sqrt(a1**2 + 1)
y_coords = abs(a2 * 0 - 0 + b2) / np.sqrt(a2**2 + 1)
print(x_coords, y_coords)
orientation = math.degrees(math.atan2(y_coords, x_coords))
print(orientation)

x_coords_1 = abs(a2 * 0 - 0 + b2) / np.sqrt(a2**2 + 1)
y_coords_1 = abs(a1 * 0 - 0 + b1) / np.sqrt(a1**2 + 1)
print(x_coords_1, y_coords_1)
orientation_1 = math.degrees(math.atan2(y_coords_1, x_coords_1))
print(orientation_1)

plt.plot(x_coords, y_coords, label='Robot Cooridnates', color='Black',marker='o')
plt.xlim([0,1])
plt.ylim([0,1])
plt.legend()
plt.show()
