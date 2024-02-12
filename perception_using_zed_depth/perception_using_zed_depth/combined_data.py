import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D

import math

class RovPose(Node):

    def __init__(self):
        super().__init__('rov_pose')
        self.right_cam_sub = self.create_subscription(
            Float32MultiArray,
            '/zed/right_data',
            self.right_cam_callback,
            10)

        self.left_cam_sub = self.create_subscription(
            Float32MultiArray,
            '/zed/left_data',
            self.left_cam_calback,
            10)
        
        self.pose_pub = self.create_publisher(Pose2D, 'rov/pose2D', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.pose_callback)
        self.i = 0

        self.right_dist = 0.0
        self.right_slope = 0.0

        self.left_dist = 0
        self.left_slope = 0

        self.right_cam_sub
        self.left_cam_sub

    def right_cam_callback(self, msg):
        self.right_dist = msg.data[0]
        self.right_slope = msg.data[1]

    def left_cam_calback(self, msg):
        self.left_dist = msg.data[0]
        self.left_slope = msg.data[1]

    def pose_callback(self):
        msg = Pose2D()

        msg.x = float(self.right_dist)
        msg.y = float(self.left_dist)
        radians = math.atan((self.left_slope + self.right_slope)/2)
        # msg.theta = (math.degrees(radians))
        msg.theta = -math.degrees(radians)
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    rov_pose = RovPose()

    rclpy.spin(rov_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rov_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()