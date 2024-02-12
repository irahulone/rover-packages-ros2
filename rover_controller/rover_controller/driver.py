import rclpy
from rclpy.node import Node

from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Pose2D

import math

class Driver(Node):

    def __init__(self):
        super().__init__('driver')
        self.pose_sub = self.create_subscription(
            Pose2D,
            'rov/pose2D',
            self.driver_callback,
            10)

        self.control_pub = self.create_publisher(Twist, 'rov/control', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback)
        self.i = 0
        
        self.lat_err = 0
        self.ang_err = 0

        self.u_lx = 0
        self.uaz = 0

        self.kp_theta = 0.1
        self.kp_lat = 0.1
        

    def driver_callback(self, msg):
        self.lat_err = 0 - (msg.x - msg.y)
        print(self.lat_err)
        self.ang_err = 0 - msg.theta

        self.u_lx = .1
        self.uaz = self.kp_theta*self.ang_err + self.kp_lat*self.lat_err

    def control_callback(self):
        msg = Twist()
        msg.linear.x = self.u_lx
        msg.angular.z = self.uaz
        
        self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    driver = Driver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()