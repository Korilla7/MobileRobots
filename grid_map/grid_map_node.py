import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from itertools import cycle
from nav_msgs.msg import Odometry

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

class ListenerNode(Node):
    pose_coords = ()
    pose_coord_ref = ()
    scan_arr = []
    initial_pose = False
    delta_coords = ()
    map_width = 120
    map_length = 120
    zero_point_x = int(map_width/2)
    zero_point_y = int(map_length/2)
    grid_map = [[50 for _ in range(map_width)] for _ in range(map_length)]

    def __init__(self):
        super().__init__('Grid_Map_Node')
        self.subscription_pose = self.create_subscription(Odometry, '/PIONIER6/RosAria/pose', self.pose_callback, 10)
        self.subscription = self.create_subscription(LaserScan, '/PIONIER6/scan', self.listener_callback, 10)
    
    def pose_callback(self, msg):
        if self.initial_pose == False:
            self.pose_coord_ref = (msg.pose.pose.position.x + 0.18, msg.pose.pose.position.y, msg.pose.pose.orientation.z)
            self.initial_pose == True
        self.pose_coords = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z)


    def listener_callback(self, msg):
        self.scan_arr = msg.ranges
        xi = np.arange(0,512)
        theta = (np.pi/512 )*(xi-256)  # angle in radians
        [x,y] = pol2cart(self.scan_arr, theta)
        data_xy = [(x, y) for x, y in zip(x, y)]
        data_xy = [(x, y) for x, y in data_xy if all(math.isfinite(val) for val in (x, y))]
        self.scan_arr = data_xy

        self.delta_coords = (self.pose_coords[0] - self.pose_coord_ref[0], self.pose_coords[1] - self.pose_coord_ref[1], self.pose_coords[2] - self.pose_coord_ref[2])
        for point in self.scan_arr:
            rotated_x = point[0] * math.cos(math.radians(self.delta_coords[2])) - point[1] * math.sin(math.radians(self.delta_coords[2]))
            rotated_y = point[0] * math.sin(math.radians(self.delta_coords[2])) + point[1] * math.cos(math.radians(self.delta_coords[2]))
            x = rotated_x + self.delta_coords[0]
            y = rotated_y + self.delta_coords[1]

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()