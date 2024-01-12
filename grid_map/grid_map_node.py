import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numbers

MAP_WIDTH = 250
MAP_LENGTH = 250
RESOLUTION = 0.08

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
    initial_pose = True
    angle_min = 0.0
    angle_inc = 0.0
    delta_coords = ()
    zero_point_x = int(MAP_WIDTH/2)
    zero_point_y = int(MAP_LENGTH/2)
    grid_map = []

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
        super().__init__('Grid_Map_Node')
        self.grid_map = [[50] * MAP_WIDTH for _ in range(MAP_LENGTH)]
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

        # xi = np.arange(0,512)
        # theta = (np.pi/512 )*(xi-256)  # angle in radians
        # [x,y] = pol2cart(self.scan_arr, theta)
        # data_xy = [(x, y) for x, y in zip(x, y)]
        # data_xy = [(x, y) for x, y in data_xy if all(math.isfinite(val) for val in (x, y))]
        # self.scan_arr = data_xy

        if len(self.pose_coords) != 0:
            self.delta_coords = (self.pose_coords[0] - self.pose_coord_ref[0], self.pose_coords[1] - self.pose_coord_ref[1], self.pose_coords[2] - self.pose_coord_ref[2])
            global_coords = []
            for point in self.scan_arr:
                rotated_x = point[0] * math.cos(math.radians(self.delta_coords[2])) - point[1] * math.sin(math.radians(self.delta_coords[2]))
                rotated_y = point[0] * math.sin(math.radians(self.delta_coords[2])) + point[1] * math.cos(math.radians(self.delta_coords[2]))
                x = rotated_x + self.delta_coords[0]
                y = rotated_y + self.delta_coords[1]
                if not math.isnan(x) and not math.isnan(y):
                    global_coords.append((x,y))
            
            for i, j in global_coords:
                x_coord = int(i/RESOLUTION) + self.zero_point_x
                y_coord = int(j/RESOLUTION) + self.zero_point_y
                line_points = bresenham_line(self.zero_point_x, self.zero_point_y, x_coord, y_coord)
                try:
                    for point in line_points:
                        if isinstance(self.grid_map[point[0]][point[1]], numbers.Number):
                            self.grid_map[point[0]][point[1]] *= 0.5
                        else:
                            pass
                except:
                    print(f"line_points {line_points}")
                    print(f"point {point}")
                    print(f"isinstance {self.grid_map[point[0]][point[1]]}")
                self.grid_map[x_coord][y_coord] = 0.95

            plt.imshow(self.grid_map, interpolation="nearest", cmap='Blues')
            plt.draw()
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()