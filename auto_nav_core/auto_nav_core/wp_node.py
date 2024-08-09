import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

import numpy
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/r1/heading',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.sub_gps_agg = self.create_subscription(
            NavSatFix,
            '/r1/gps_agg',
            self.gps_agg_cb,
            10)
        self.sub_gps_agg  # prevent unused variable warning

        self.pub_cmdvel = self.create_publisher(Twist, '/r3/cmd_vel', 10)

        self.r_heading = 0.0
        self.des_heading = 0.0
        self.wp_distance = 0.0

        self.r_lat = 0.0
        self.r_lon = 0.0

        self.des_lat = 37.26045465
        self.des_lon = -121.83940795

    def listener_callback(self, msg):
        self.r_heading = msg.data
        #self.get_logger().info('I heard: "%s"' % msg.data)
        print(self.r_heading)

        self.des_heading = self.get_bearing(self.r_lat, self.r_lon, self.des_lat, self.des_lon)
        print(self.des_heading)

        self.wp_distance = self.get_distance(self.r_lat, self.r_lon, self.des_lat, self.des_lon)
        print(self.wp_distance)
        print(" ")

        e_h = self.des_heading - self.r_heading 
        e_d = self.wp_distance

        uaz = 0.06*e_h
        if uaz > 0.6:
            uaz = 0.6
        elif uaz < -0.6:
            uaz = -0.6
        
        ulx = 0.2*e_d
        if ulx > 0.4:
            ulx = 0.4
        elif ulx < -0.4:
            ulx = -0.4

        msg_cmd = Twist()
        msg_cmd.linear.x = ulx
        msg_cmd.angular.z = uaz
        self.pub_cmdvel.publish(msg_cmd)


    def gps_agg_cb(self, msg):
        self.r_lat = msg.latitude
        self.r_lon = msg.longitude

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x,y)
        brng = 90 -  numpy.degrees(brng) - 88
        if (brng < 0):
            brng = 360 + brng
        return brng


    def get_distance(self, lat1, long1, lat2, long2):
        dLat = (math.radians(lat2) - math.radians(lat1))
        dLon = (math.radians(long2) - math.radians(long1))
        R = 6373.0
        a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R*c*1000
        return distance


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
