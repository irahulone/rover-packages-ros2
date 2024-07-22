import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class JointStates(Node):

    def __init__(self):
        super().__init__('teleop_to_jointstate')
        self.subscription = self.create_subscription(
            Twist,
            'r2/joy/cmd_vel',
            self.teleop_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.lx = 0
        self.az = 0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)

    def teleop_callback(self, msg):
        self.lx = msg.linear.x
        self.az = msg.angular.z
        self.pos_x = self.pos_x + 0.02*self.lx
        self.pos_theta = self.pos_theta + 0.02*self.az
        self.pos_y =self.pos_x*math.tan(self.pos_theta)

        jointstates_msg = JointState()
        t = time.time()
        #jointstates_msg.header.stamp.sec = math.trunc(time.time())
        jointstates_msg.header.stamp.sec = int(t//1)
        jointstates_msg.header.stamp.nanosec = int((t%1)*math.pow(10,9)//1)
        #jointstates_msg.header.frame_id = '_'
        jointstates_msg.name = ['w_to_x', 'x_to_y', 'y_to_t']
        #jointstates_msg.name = ['w_to_x']
        jointstates_msg.position = [self.pos_x, self.pos_y,  self.pos_theta]
        self.joint_state_publisher_.publish(jointstates_msg)


def main(args=None):
    rclpy.init(args=args)

    teleop_sub = JointStates()

    rclpy.spin(teleop_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()