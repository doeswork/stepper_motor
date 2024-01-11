import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, 'stepper_motor_command', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {command}")

def main(args=None):
    rclpy.init(args=args)
    input_node = InputNode()

    while rclpy.ok():
        command = input("Enter command (L/R): ")
        if command in ['L', 'R']:
            input_node.publish_command(command)
        else:
            print("Invalid command")

    input_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

