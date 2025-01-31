import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from action import waypoint


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__('waypoint_action_server')
        self._action_server = ActionServer(
            self,
            waypoint,
            'waypoint_action',
            self.execute_callback)

        self.ref_coords1 = self.create_publisher(NavSatFix, '/r1/ref_coordinate1', 5)
        self.ref_coords2 = self.create_publisher(NavSatFix, '/r1/ref_coordinate2', 5)

        self.create_subscription(Float32, '/r1/dist_to_goal_pose', self.dist_to_goal_callback, 5)
        self.create_subscription(NavSatFix,'/r1/gps_agg',self.gps_agg_cb, 5)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        points = goal_handle.request.target

        nav_point_start = points[0]
        nav_point_end = points[1]
        
        self.ref_coords1.publish(nav_point_start)
        self.ref_coords2.publish(nav_point_end)

        while True:
            if self.dist is None:
                await self.get_clock().sleep_for(rclpy.time.Duration(seconds=1))
                continue

            feedback_msg = waypoint.Feedback()
            feedback_msg.dist = self.dist
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Remaining distance: {feedback_msg.dist} meters")

            if self.dist < 0.1:
                self.get_logger().info("Waypoint Reached")
                break

            await self.get_clock().sleep_for(rclpy.time.Duration(seconds=1))


        goal_handle.succeed()

        result = waypoint.Result()
        result.current_position = NavSatFix(latitude=self.r_lat, longitude=-self.r_lon)
        return result

    def dist_to_goal_callback(self, msg):
        self.dist = msg.data

    def gps_agg_cb(self, msg):
        self.r_lat = msg.latitude
        self.r_lon = msg.longitude


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = WaypointActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()