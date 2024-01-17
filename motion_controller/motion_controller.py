import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from numpy import float32
import time
import math

class VelocityPublisher(Node):
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
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/pioneer5/cmd_vel', 10)
        self.get_logger().info('Velocity Publisher has been started.')

        self.pose_subscriber = self.create_subscription(Odometry, '/PIONIER6/RosAria/pose', self.pose_callback, 10)
        self.get_logger().info('Pose subscriber has been started.')

        self.path_subscriber = self.create_subscription(Path, '/seba_niko/path', self.path_callback, 10)
        self.get_logger().info('Path subscriber has been started.')

        
        self.grid_path = Path()
        self.current_coordinate_index = 0
        self.new_path_flag = 0
        self.initial_position = True
        self.current_position = (0, 0)
        self.position_ref = (0,0)
        self.initial_yaw = True
        self.current_yaw = 0
        self.yaw_ref = 0

        # PI controller parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.error_integral = 0.0

        # Timer to periodically control the robot
        self.timer = self.create_timer(1.0, self.control_robot)

    def publish_velocity(self,v,w):
        velocity_command = Twist()
        velocity_command.linear.x = v
        velocity_command.angular.z = w

        self.publisher_.publish(velocity_command)
        self.get_logger().info('Published velocity command: linear={}, angular={}'.format(
            velocity_command.linear.x, velocity_command.angular.z))

    def path_callback(self, msg):
        # self.new_path_flag = 1
        self.grid_path = msg
        self.get_logger().info('Received path: {}'.format(msg))

    def pose_callback(self, msg):
        if self.initial_yaw:
            self.yaw_ref =  math.degrees(self.yaw_from_quaternion(msg.pose.pose.orientation))
            self.initial_yaw = False
        self.current_yaw = math.degrees(self.yaw_from_quaternion(msg.pose.pose.orientation))
        
        if self.initial_position:
            self.position_ref =  (msg.pose.pose.position.x + 0.18, msg.pose.pose.position.y)
            self.initial_position = False
        self.current_position = (msg.pose.pose.position.x + 0.18, msg.pose.pose.position.y)
    
    def control_robot(self):
        target_position = Point()
        # target_position.x, target_position.y = self.grid_path[self.current_coordinate_index]

        target_position.x = self.grid_path.poses[self.current_coordinate_index].pose.position.x
        target_position.y = self.grid_path.poses[self.current_coordinate_index].pose.position.y

        # Calculate linear velocity (assuming constant speed)
        linear_velocity = 0.5

        # Calculate angular error
        target_angle = math.atan2(target_position.y - self.current_position[0], target_position.x - self.current_position[1])
        angular_error = target_angle - self.current_yaw

        # Update integral error
        self.error_integral += angular_error

        # Calculate angular velocity using PI controller
        angular_velocity = self.kp * angular_error + self.ki * self.error_integral

        # Publish velocity commands
        self.publish_velocity(linear_velocity, angular_velocity)

        # Check if the robot has reached the target coordinate
        distance_to_target = math.sqrt((target_position.x - self.current_position[0])**2 +
                                       (target_position.y - self.current_position[1])**2)
        if distance_to_target < 0.1:
            self.current_coordinate_index += 1
            if self.current_coordinate_index == len(self.grid_path):
                self.timer.cancel()  # Stop the controller when all coordinates are reached

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    # time.sleep(8)
    velocity_publisher.get_logger().info('Start')
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()