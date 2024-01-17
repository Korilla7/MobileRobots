
import numpy as np
import json
import math
import copy
import matplotlib.pyplot as plt

MAP_WIDTH = 50 
# MAP_WIDTH = 250
MAP_LENGTH = 50 
# MAP_LENGTH = 250
RESOLUTION = 0.5
# RESOLUTION = 0.1
GOAL_X = -2
# GOAL_X = -4
GOAL_Y = 4
# GOAL_Y = 6

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x,y)

# Implement wavefront planning
def wavefront_planning(path_map, start):
    goal_cell = (int(GOAL_X / RESOLUTION) + int(MAP_WIDTH / 2), int(GOAL_Y / RESOLUTION) + int(MAP_LENGTH / 2))
    wavefront_map = copy.deepcopy(path_map)
    wavefront_map[goal_cell[0]][goal_cell[1]] = 0
    i = 0
    while wavefront_map[start[0]][start[1]] is None:
        for x in range(MAP_WIDTH):
            for y in range(MAP_LENGTH):
                if wavefront_map[x][y] == i:
                    neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
                    for nx, ny in neighbors:
                        if 0 <= nx < MAP_WIDTH and 0 <= ny < MAP_LENGTH and wavefront_map[nx][ny] is None:
                            wavefront_map[nx][ny] = i + 1
        i += 1

    current = start
    path = [current]
    while current != goal_cell:
        neighbors = [(current[0]-1, current[1]), (current[0]+1, current[1]), (current[0], current[1]-1), (current[0], current[1]+1)]
        valid_neighbors = [(nx, ny) for nx, ny in neighbors if 0 <= nx < MAP_WIDTH and 0 <= ny < MAP_LENGTH and wavefront_map[nx][ny] is not None]
        if valid_neighbors:
            next_cell = min(valid_neighbors, key=lambda x: wavefront_map[x[0]][x[1]])
            current = next_cell
            path.append(current)
        else:
            # print("No valid neighbors found. Stopping the wavefront planning.")
            break
    return path 



json_data = open('./map_big.json')
data_loaded = json.load(json_data)

pose_arr = []
path_map_list = []

for data_sample in data_loaded:
    pose_arr.append(data_sample["pose"])
    path_map_list.append(data_sample["scan"])

i = 0
for data in path_map_list:
    xi = np.arange(0, 512)
    theta = (np.pi / 512) * (xi - 256)  # angle in radians
    [x, y] = pol2cart(data, theta)
    data_xy = [(x, y) for x, y in zip(x, y)]
    data_xy = [(x, y) for x, y in data_xy if all(math.isfinite(val) for val in (x, y))]
    path_map_list[i] = data_xy
    i=i+1

ref_x = pose_arr[0][0] + 0.18
ref_y = pose_arr[0][1]
ref_theta = pose_arr[0][2]


zero_point_x = int(MAP_WIDTH / 2)
zero_point_y = int(MAP_LENGTH / 2)
scans_in_global_x = []
scans_in_global_y = []


for data_scan, data_pose in zip(path_map_list, pose_arr):
    delta_x = data_pose[0] - ref_x
    delta_y = data_pose[1] - ref_y
    delta_theta = data_pose[2] - ref_theta
    start_x= int(delta_x / RESOLUTION) + zero_point_x
    start_y = int(delta_y / RESOLUTION) + zero_point_y
    for x, y in data_scan:
        rotated_x = x * math.cos(math.radians(delta_theta)) - y * math.sin(math.radians(delta_theta))
        rotated_y = x * math.sin(math.radians(delta_theta)) + y * math.cos(math.radians(delta_theta))
        x_global = rotated_x + delta_x
        y_global = rotated_y + delta_y
        scans_in_global_x.append(x_global)
        scans_in_global_y.append(y_global)

    path_map = [[None for _ in range(MAP_WIDTH)] for _ in range(MAP_LENGTH)]
    path_map_fake = [[None for _ in range(MAP_WIDTH)] for _ in range(MAP_LENGTH)]

    unique_points = set(zip(scans_in_global_x, scans_in_global_y))
    for i, j in unique_points:
        x_coord = int(i / RESOLUTION) + zero_point_x
        y_coord = int(j / RESOLUTION) + zero_point_y
        path_map[x_coord][y_coord] = np.inf
        path_map_fake[x_coord][y_coord] = np.inf

        # #Making artificial obstacles
        # if 1 < x_coord < MAP_WIDTH-2:
        #     path_map_fake[x_coord+1][y_coord] = np.inf
        #     path_map_fake[x_coord-1][y_coord] = np.inf
        #     path_map_fake[x_coord+2][y_coord] = np.inf
        #     path_map_fake[x_coord-2][y_coord] = np.inf
        # if 1 < y_coord < MAP_LENGTH-2:
        #     path_map_fake[x_coord][y_coord+1] = np.inf
        #     path_map_fake[x_coord][y_coord-1] = np.inf
        #     path_map_fake[x_coord][y_coord+2] = np.inf
        #     path_map_fake[x_coord][y_coord-2] = np.inf
        # if 1 < x_coord < MAP_WIDTH-2 and 1 < y_coord < MAP_LENGTH-2:
        #     path_map_fake[x_coord-1][y_coord+1] = np.inf
        #     path_map_fake[x_coord-1][y_coord-1] = np.inf
        #     path_map_fake[x_coord+1][y_coord+1] = np.inf
        #     path_map_fake[x_coord+1][y_coord-1] = np.inf

    path = wavefront_planning(path_map_fake, (start_x, start_y))

    for x in range(MAP_WIDTH):
        for y in range(MAP_LENGTH):
            if (x,y) in path:
                path_map[x][y] = 50
            elif path_map[x][y] is None:
                path_map[x][y] = 10
            else:
                path_map[x][y] = 100

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    # cmap = plt.cm.get_cmap('viridis', 3)
    # plt.imshow(path_map, cmap=cmap, interpolation='nearest')
    plt.imshow(path_map, interpolation="nearest", cmap='Greys')
    plt.pause(0.1)

plt.colorbar()
plt.show()
