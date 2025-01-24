import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from actions import planter
from motor_controller import MotorNode
from ser import Serial


class PlanterActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            planter,
            'planter_action',
            self.execute_callback)

        #conver belt shit
        conv_motor_1 = MotorNode("1", "/dev/ttyACM1")
        # conv_motor_2 = MotorNode("2", "/dev/ttyACM1")
        # conv_motor_3 = MotorNode("3", "/dev/ttyACM1")

        self.ref_coords1 = self.create_publisher(NavSatFix, '/r1/ref_coordinate1', 5)
        self.ref_coords2 = self.create_publisher(NavSatFix, '/r1/ref_coordinate2', 5)
        self.motor_1_speed = self.create_publisher(Int16, 'motor_1/set_speed', 5)

        self.create_subscription(Float32, '/r1/dist_to_goal_pose', self.dist_to_goal_callback, 5)
        self.create_subscription(NavSatFix,'/r1/gps_agg',self.gps_agg_cb, 5)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.phase_1()
        self.phase_2()

        goal_handle.succeed()

        result = planter.Result()
        result.current_position = 1
        return result

    def stage_1(self):
        self.motor_1_speed.publish()

        

        return

    def stage_2(self):
        #activate plant

        return

    def getSensorData():
        

    def dist_to_goal_callback(self, msg):
        self.dist = msg.data

    def gps_agg_cb(self, msg):
        self.r_lat = msg.latitude
        self.r_lon = msg.longitude


def main(args=None):
    rclpy.init(args=args)

    planter_action_server = PlanterActionServer()

    rclpy.spin(planter_action_server)


if __name__ == '__main__':
    main()