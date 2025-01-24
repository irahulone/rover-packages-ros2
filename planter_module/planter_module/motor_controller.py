#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

global serialFlag
serialFlag = 1

import serial

class MotorNode(Node):
    def __init__(self, motor_id, motor_port):
        super().__init__(f'motor_{motor_id}_node')

        self.roboteq_obj = serial.Serial(
            port=motor_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)

        self.motor_id = motor_id
        self.speed_subscriber = self.create_subscription(
            Int16,
            f'motor_{motor_id}/set_speed',
            self.speed_callback,
            10
        )
        self.get_logger().info(f'Motor {self.motor_id} node initialized.')

    def speed_callback(self, msg):
        inCmd = msg.data
        global serialFlag
        payload1 = "!G " + str(self.motor_id) + " " + str(-inCmd) + "_"
        payload2 = "!G " + str(self.motor_id) + " " + str(-inCmd) + "_"
        #if(serialFlag):
        self.roboteq_obj.write(str.encode(payload1))
        self.roboteq_obj.write(str.encode(payload2))


def main(args=None):
    rclpy.init(args=args)
    motor_id = int(args[1]) if len(args) > 1 else 0  # Get motor ID from args
    motor_node = MotorNode(motor_id)
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()
