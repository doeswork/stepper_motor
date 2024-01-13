import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Stepper motor setup
        self.control_pins = [17, 18, 27, 22]
        GPIO.setmode(GPIO.BCM)
        for pin in self.control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
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

        # Servo motor setup 1
        self.servo_pin1 = 4 # Use a different pin for the servo
        GPIO.setup(self.servo_pin1, GPIO.OUT)
        self.servo1 = GPIO.PWM(self.servo_pin1, 50)  # 50Hz pulse for servo
        self.servo1.start(0)  # Initialization

        # Servo motor setup 2
        self.servo_pin2 = 5 # Use a different pin for the servo
        GPIO.setup(self.servo_pin2, GPIO.OUT)
        self.servo2 = GPIO.PWM(self.servo_pin2, 50)  # 50Hz pulse for servo
        self.servo2.start(0)  # Initialization

        # Servo motor setup 3
        self.servo_pin3 = 6 # Use a different pin for the servo
        GPIO.setup(self.servo_pin3, GPIO.OUT)
        self.servo3 = GPIO.PWM(self.servo_pin3, 50)  # 50Hz pulse for servo
        self.servo3.start(0)  # Initialization

        # Servo motor setup 4
        self.servo_pin4 = 13 # Use a different pin for the servo
        GPIO.setup(self.servo_pin4, GPIO.OUT)
        self.servo4 = GPIO.PWM(self.servo_pin4, 50)  # 50Hz pulse for servo
        self.servo4.start(0)  # Initialization

        # Subscription
        self.subscription = self.create_subscription(
            String,
            'stepper_motor_command',
            self.command_callback,
            10)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command == 'L':
            self.rotate_motor('left', 512)
        elif command == 'R':
            self.rotate_motor('right', 512)
        elif command.startswith('S'):
            parts = command.split(':')
            angle = int(parts[1])
            servo_number = int(parts[0][1:])
            if servo_number == 1:
                servo = self.servo1
            elif servo_number == 2:
                servo = self.servo2
            elif servo_number == 3:
                servo = self.servo3
            elif servo_number == 4:
                servo = self.servo4
            else:
                print("Invalid servo number")
                servo = None

            self.set_servo_angle(angle, servo)

    def rotate_motor(self, direction, steps):
        for i in range(steps):
            for step in range(8):
                for pin in range(4):
                    GPIO.output(self.control_pins[pin], self.seq[step][pin] if direction == 'left' else self.seq[7 - step][pin])
                time.sleep(0.001)

    def set_servo_angle(self, angle, servo):
        duty = float(angle) / 18.0 + 2.5
        servo.ChangeDutyCycle(duty)

    def __del__(self):
        self.servo1.stop()
        self.servo2.stop()
        self.servo3.stop()
        self.servo4.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
