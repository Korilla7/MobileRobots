import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/pioneer5/cmd_vel', 10)
        self.get_logger().info('Velocity Publisher has been started.')

    def publish_velocity(self,v,w):
        velocity_command = Twist()
        velocity_command.linear.x = v  # Replace this with your desired linear velocity
        velocity_command.angular.z = w  # Replace this with your desired angular velocity

        self.publisher_.publish(velocity_command)
        self.get_logger().info('Published velocity command: linear={}, angular={}'.format(
            velocity_command.linear.x, velocity_command.angular.z))

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    time.sleep(8)
    velocity_publisher.get_logger().info('Start')
    try:
        velocity_publisher.publish_velocity(0.0,0.5)
        time.sleep(2)
        velocity_publisher.publish_velocity(0.0,-0.5)
        time.sleep(2)
        velocity_publisher.publish_velocity(0.0,0.0)
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()