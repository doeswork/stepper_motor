import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node

class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor')
        
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

        # Call rotate_motor method
        self.rotate_motor('left', 512)  # Rotate left
        time.sleep(2)  # Wait for 2 seconds
        self.rotate_motor('right', 512)  # Rotate right

        GPIO.cleanup()

    # Function to rotate the motor
    def rotate_motor(self, direction, steps):
        for i in range(steps):
            for step in range(8):
                for pin in range(4):
                    GPIO.output(self.control_pins[pin], self.seq[step][pin] if direction == 'left' else self.seq[7 - step][pin])
                time.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    stepper_motor_node = StepperMotorNode()
    rclpy.spin(stepper_motor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

