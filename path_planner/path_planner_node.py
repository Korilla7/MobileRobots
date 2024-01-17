import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numbers
import copy

# MAP_WIDTH = 50 
MAP_WIDTH = 250
# MAP_LENGTH = 50 
MAP_LENGTH = 250
# RESOLUTION = 0.5
RESOLUTION = 0.1
# GOAL_X = -2
GOAL_X = -4
# GOAL_Y = 4
GOAL_Y = 6

ZERO_POINT_X = int(MAP_WIDTH / 2)
ZERO_POINT_Y = int(MAP_LENGTH / 2)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x,y)

class ListenerNode(Node):
    pose_coords = ()
    pose_coord_ref = ()
    path_map = []
    path_map_fake = []
    initial_pose = True
    angle_min = 0.0
    angle_inc = 0.0
    delta_coords = ()
    goal = (int(GOAL_X / RESOLUTION) + ZERO_POINT_X, (int(GOAL_Y / RESOLUTION) + ZERO_POINT_Y))
    start = ()
    global_coords = []

    def yaw_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def __init__(self):
        super().__init__('Path_Planner_Node')
        self.path_map = [[None for _ in range(MAP_WIDTH)] for _ in range(MAP_LENGTH)]
        self.path_map_fake = [[None for _ in range(MAP_WIDTH)] for _ in range(MAP_LENGTH)]

        self.subscription_pose = self.create_subscription(Odometry, '/PIONIER6/RosAria/pose', self.pose_callback, 10)
        self.subscription = self.create_subscription(LaserScan, '/PIONIER6/scan', self.listener_callback, 10)
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        
    def pose_callback(self, msg):
        if self.initial_pose:
            self.pose_coord_ref = (msg.pose.pose.position.x + 0.18, msg.pose.pose.position.y, math.degrees(self.yaw_from_quaternion(msg.pose.pose.orientation)))
            self.initial_pose = False
        self.pose_coords = (msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(self.yaw_from_quaternion(msg.pose.pose.orientation)))

    def wavefront_planning(self):
        goal_cell = (int(GOAL_X / RESOLUTION) + ZERO_POINT_X, int(GOAL_Y / RESOLUTION) + ZERO_POINT_Y)
        wavefront_map = copy.deepcopy(self.path_map_fake)
        wavefront_map[goal_cell[0]][goal_cell[1]] = 0
        wavefront_map[self.start[0]][self.start[1]] = None
        i = 0
        while wavefront_map[self.start[0]][self.start[1]] is None:
            for x in range(MAP_WIDTH):
                for y in range(MAP_LENGTH):
                    if wavefront_map[x][y] == i:
                        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
                        for nx, ny in neighbors:
                            if 0 <= nx < MAP_WIDTH and 0 <= ny < MAP_LENGTH and wavefront_map[nx][ny] is None:
                                wavefront_map[nx][ny] = i + 1
            i += 1

        current = self.start
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

    def listener_callback(self, msg):
        self.scan_arr = []
        ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

        data_xy = []
        for i in range(len(ranges)):
            if np.isnan(ranges[i]):
               data_xy.append([0, self.angle_min + i * self.angle_inc])
            else:
                data_xy.append(
                    [ranges[i], self.angle_min + i * self.angle_inc]
                )
        self.scan_arr = [pol2cart(x[0], x[1]) for x in data_xy]

        if len(self.pose_coords) != 0:
            self.global_coords = []
            self.delta_coords = (self.pose_coords[0] - self.pose_coord_ref[0], self.pose_coords[1] - self.pose_coord_ref[1], self.pose_coords[2] - self.pose_coord_ref[2])
            start_x = int(self.delta_coords[0] / RESOLUTION) + ZERO_POINT_X
            start_y = int(self.delta_coords[1] / RESOLUTION) + ZERO_POINT_Y
            self.start = (start_x, start_y)
            
            for point in self.scan_arr:
                rotated_x = point[t0] * math.cos(math.radians(self.delta_coords[2])) - point[1] * math.sin(math.radians(self.delta_coords[2]))
                rotated_y = point[0] * math.sin(math.radians(self.delta_coords[2])) + point[1] * math.cos(math.radians(self.delta_coords[2]))
                x = rotated_x + self.delta_coords[0]
                y = rotated_y + self.delta_coords[1]
                if not math.isnan(x) and not math.isnan(y):
                    self.global_coords.append((x,y))
            
            self.path_map = [[None for _ in range(MAP_WIDTH)] for _ in range(MAP_LENGTH)]
            self.path_map_fake = copy.deepcopy(self.path_map)

            for i, j in self.global_coords:
                x_coord = int(i/RESOLUTION) + ZERO_POINT_X
                y_coord = int(j/RESOLUTION) + ZERO_POINT_Y
                self.path_map[x_coord][y_coord] = np.inf
                self.path_map_fake[x_coord][y_coord] = np.inf

            path = self.wavefront_planning()
            
            for x in range(MAP_WIDTH):
                for y in range(MAP_LENGTH):
                    if (x,y) in path:
                        self.path_map[x][y] = 50
                    elif self.path_map[x][y] is None:
                        self.path_map[x][y] = 10
                    else:
                        self.path_map[x][y] = 100


            plt.imshow(self.path_map, interpolation="nearest", cmap='Greys')
            plt.pause(0.1)

            # for x in range(MAP_WIDTH):
            #     for y in range(MAP_LENGTH):
            #         if (x,y) in path:
            #             self.path_map[x][y] = None
            #         elif self.path_map[x][y] == 10:
            #             self.path_map[x][y] = None
            #         else:
            #             self.path_map[x][y] = np.inf
            
def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()