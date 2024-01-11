import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            String,
            'stepper_motor_command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Define the GPIO pins
        self.control_pins = [17, 18, 27, 22]

        # Use the Broadcom SOC Pin numbers
        GPIO.setmode(GPIO.BCM)

        # Set up the control pins
        for pin in self.control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        # Define a sequence to rotate the motor
        self.seq = [
            [1, 0, 0, 1],
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1]
        ]

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command == 'L':
            # Rotate motor left
            self.rotate_motor('left', 512)
        elif command == 'R':
            # Rotate motor right
            self.rotate_motor('right', 512)

    # Function to rotate the motor
    def rotate_motor(self, direction, steps):
        for i in range(steps):
            for step in range(8):
                for pin in range(4):
                    GPIO.output(self.control_pins[pin], self.seq[step][pin] if direction == 'left' else self.seq[7 - step][pin])
                time.sleep(0.001)

    def __del__(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


