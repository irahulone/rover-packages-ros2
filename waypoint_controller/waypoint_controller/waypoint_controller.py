#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import numpy
import time


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_heading_controller')

        # Global variables initialization
        self.rover_heading = 0.0
        self.ref_heading = 10.0
        self.heading_error_i = 0.0

        self.rover_lat = 0.0
        self.rover_lon = 0.0

        self.ref_coord_1_lat = 0.1
        self.ref_coord_1_lon = 0.12
        self.ref_coord_2_lat = 0.13
        self.ref_coord_2_lon = 0.14

        self.history = []
        self.f1 = 0
        self.path_id = 0
        self.cmd = Twist()

        # Publishers
        self.pub_rover_cmdvel = self.create_publisher(Twist, 'AGBOT1_cmd_vel', 5)
        self.pub_ref_heading = self.create_publisher(Float32, '/r1/ref_heading', 5)
        self.pub_path_bearing = self.create_publisher(Float32, '/r1/path_bearing', 5)
        self.pub_xte = self.create_publisher(Float32, '/r1/xte', 5)
        self.pub_exe_status = self.create_publisher(Bool, '/r1/exe_status', 5)
        self.pub_dost_to_goal = self.create_publisher(Float32, '/r1/dist_to_goal_pose', 5)
        self.pub_rover_pos = self.create_publisher(NavSatFix, '/r1/rover_gps', 5)

        # Subscribers
        self.create_subscription(Float32, 'ar1_heading', self.rover_heading_callback, 5)
        self.create_subscription(Int16, 'r1/path_id', self.path_id_callback, 5)
        self.create_subscription(NavSatFix,'/r1/gps_agg',self.gps_agg_cb, 10)
        self.create_subscription(NavSatFix, '/r1/ref_coordinate1', self.ref_coord1_callback, 5)
        self.create_subscription(NavSatFix, '/r1/ref_coordinate2', self.ref_coord2_callback, 5)

        # Timer to simulate ROS1 rate loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def rover_heading_callback(self, msg):
        self.rover_heading = msg.data

    def gps_agg_cb(self, msg):
        self.r_lat = msg.latitude
        self.r_lon = msg.longitude

    def ref_coord1_callback(self, msg):
        self.ref_coord_1_lat = msg.latitude
        self.ref_coord_1_lon = msg.longitude

    def ref_coord2_callback(self, msg):
        self.ref_coord_2_lat = msg.latitude
        self.ref_coord_2_lon = msg.longitude

    def path_id_callback(self, msg):
        self.path_id = msg.data

    def saturation_fn(self, val, upper_bound, lower_bound):
        x = val
        if x > upper_bound:
            x = upper_bound
        elif x < lower_bound:
            x = lower_bound
        return x

    def get_bearing(self, lat1, lon1, lat2, lon2):
        dLon = (lon2 - lon1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
            math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x, y)
        brng = numpy.degrees(brng)
        if brng < 0:
            brng = 360 + brng
        return brng

    def get_distance(self, lat1, lon1, lat2, lon2):
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        R = 6373.0
        a = math.sin(dLat / 2)**2 + math.cos(math.radians(lat1)) * \
            math.cos(math.radians(lat2)) * math.sin(dLon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c * 1000
        return distance

    def control_loop(self):
        # Main control loop logic
        # (Adapt the ROS1 loop into here; logic remains largely unchanged)
        rover_lat = self.r_lat
        rover_lon = self.r_lon
        roverGPS = NavSatFix()
        roverGPS.latitude = rover_lat
        roverGPS.longitude = rover_lon

        latitudes_field   = [self.ref_coord_1_lat, self.ref_coord_2_lat]
        longitudes_field = [self.ref_coord_1_lon, self.ref_coord_2_lon]

        path_bearing = self.get_bearing(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_bw_pts  =  self.get_distance(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_to_goal_pose = self.get_distance(rover_lat, rover_lon, latitudes_field[1], longitudes_field[1]) # in meters
        
        
        path_bearing = 360 - path_bearing
        #if(path_bearing < 0):
        #    path_bearing = 360 + path_bearing
        
        #rospy.loginfo(path_bearing)

        dy = (latitudes_field[1]- latitudes_field[0])
        dx = (longitudes_field[1]-longitudes_field[0])
        m = dy/dx
        #m = 1
        c = latitudes_field[0] - m*longitudes_field[0]
        a = m
        b = -1.0
        x1 = (rover_lon)
        y1 = (rover_lat)
        d = (a*x1 + b*y1 + c) / (math.sqrt(a*a + b*b))
        xte = d*100000

        ref_heading = path_bearing - 10*xte

        if (ref_heading < 0):
            ref_heading = 360 + ref_heading
        if (ref_heading > 360):
            ref_heading = ref_heading - 360
        
        #print(ref_heading)
        
        # linear motion controller to move rover to goal position
        #jj = 
        #kk_lx = 
        #kp_lx = 0.2
        #ux_raw = kp_lx*dist_to_goal_pose
        #ux = saturation_fn(ux_raw, 0.22, -0.22)
        #ux = 0.14

        # run status
        error_margin = 0.5
        if (dist_bw_pts < 14):
            error_margin = 1.5

        if (dist_to_goal_pose < error_margin) and (f1 == 0):
            run_status_flag = True
            ux = 0
            f1 = 1
            time.sleep(1)
        else:
            f1 = 0
            run_status_flag = False  # really False
        
        error_heading = ref_heading - self.rover_heading
        print('bferr_heading', error_heading) 
        if (error_heading <= 360) and (error_heading > 180):
            error_heading = -(360-error_heading)
        if (error_heading < -180) and (error_heading > -360):
            error_heading = 360 + error_heading
        #elif (error_heading < -180):
        #    error_heading = error_heading + 360
        #if(rover_heading > 180) and (error_heading <= 360):
        #    error_heading = -error_heading
        
        #rospy.loginfo(path_id)
        #rospy.loginfo(ref_heading)

        k_p = 0.008
        #k_i = 0.0001
        k_i = 0.0003
        if abs(error_heading > 30): 
            k_i = 0
        #rospy.loginfo(dist_to_goal_pose)
        
        heading_error_i = heading_error_i + error_heading
        
        if (abs(error_heading) < 1):
            heading_error_i = 0
        if (abs(k_i*heading_error_i) > 0.2):
            heading_error_i = 0.2
       

        # linear motion controller to move rover to goal position
        kk_lx = 1.0 - self.saturation_fn( (0.02*abs(error_heading)) , 1.0, 0.0)
        kp_lx = 0.3
        ux_raw = kp_lx*dist_to_goal_pose
        ux = kk_lx*self.saturation_fn(ux_raw, 0.22, -0.22)
        #ux = 0.14


        print(ux)
        print('pth_heading', path_bearing)
        print('ref_heading', ref_heading)
        print('ar1_heading', self.rover_heading)
        print('err_heading', error_heading)
        print('err_ii',k_i* heading_error_i)
        print('dist', dist_to_goal_pose)

        #print('err_heading', error_heading)
        
        #if (path_id == 2):
        #    k_p = -0.0035
        
        uz_raw = (k_p*error_heading + k_i*heading_error_i)
        
        #if(error_heading <= 0) and (error_heading < -180):
        #    uz_raw = (k_p*error_heading + k_i*heading_error_i)
        
        #if ux_raw < 0:
        #    uz_raw = -uz_raw
        
        uz = self.saturation_fn(uz_raw, 0.40, -0.40)
    
        #uz = uz_raw
        #if uz_raw > 0.2:
        #    uz = 0.2
        #if uz_raw < -0.2:
        #    uz = -0.2
        
        self.cmd.linear.x = ux
        self.cmd.angular.z = uz
        
        #################### PRINT ##################
        
        rclpy.loginfo(xte)
        rclpy.loginfo(uz)
        #rospy.loginfo(error_heading)

        # wait for meaningful data to arrive
        if (self.rover_heading == 1000.0):
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0
        
        self.pub_rover_cmdvel.publish(self.cmd)
        self.pub_ref_heading.publish(ref_heading)
        self.pub_path_bearing.publish(path_bearing)
        self.pub_xte.publish(xte)
        self.pub_exe_status.publish(run_status_flag)
        self.pub_dost_to_goal.publish(dist_to_goal_pose)
        self.pub_rover_pos.publish(roverGPS)


def main(args=None):
    rclpy.init(args=args)
    waypoint_controller = WaypointController()
    try:
        rclpy.spin(waypoint_controller)
    except KeyboardInterrupt:
        waypoint_controller.get_logger().info('Shutting down node...')
    finally:
        waypoint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
