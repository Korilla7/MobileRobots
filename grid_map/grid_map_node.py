import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GridMap(Node):
    def __init__(self):
        super().__init__('grid_map')

        self.subscription = self.create_subscription(Float64, '/pioneerX', self.callback, 10)

    def callback(self, msg):
        scan_data = msg.data.scan
        
def main():
    rclpy.init()
    node = GridMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()