import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

from ublox_gps import UbloxGps
import serial
port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
gps = UbloxGps(port)

class read_gps_data(Node):

    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(NavSatFix, '/r4/gps1', 2)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.gps_callback)
        self.i = 0

    def gps_callback(self):
        gps1_lat = 0.0
        gps1_lon = 0.0
        try:
            coords = gps.geo_coords()
            gps1_lat = coords.lat
            gps1_lon = coords.lon

            #print(coords.lon, coords.lat)
        except (ValueError, IOError) as err:
            print(err)


        msg = NavSatFix()
        msg.latitude = gps1_lat
        msg.longitude = gps1_lon
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pub = read_gps_data()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
