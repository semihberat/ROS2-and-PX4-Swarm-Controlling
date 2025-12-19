#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy

class JoyTester(Node):

    def __init__(self):
        super().__init__('test_joy')
        self.get_logger().info('Testing Joystick...')

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 5)
        self.subscription  # prevent unused variable warning

        self.buttons = []
        self.axes = []
        self.initialised = False

    def joy_callback(self, joy_msg):
        self.get_logger().info(f'Received Joy message: {joy_msg.buttons}, {joy_msg.axes}')
     

        return


def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()

    try:
        rclpy.spin(joy_tester)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')

    print('Exiting...')

    joy_tester.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
