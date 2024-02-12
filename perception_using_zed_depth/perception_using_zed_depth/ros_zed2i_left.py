import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import pyzed.sl as sl
import argparse
import os
import math
import numpy as np
import sys


serial_number = 31906960

zed = sl.Camera()

init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.set_from_serial_number(serial_number)
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(err)
    exit(1)
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.confidence_threshold = 100
runtime_parameters.texture_confidence_threshold = 100

# Capture 150 images and depth, then stop
i = 0
image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()

mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
tr_np = mirror_ref.m

class MinimalPublisher(Node):

    

    def __init__(self):
        super().__init__('zed_l')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'zed/left_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.filter_size = 10
        self.distance_queue = []
        self.slope_queue = []

    def timer_callback(self):

        def moving_filter(queue, filter_size):
            print("{} size: {}".format(queue, len(queue)))
            if(len(queue) == filter_size):
                avg = float(sum(queue)/filter_size)
                queue.pop(0)
                print(type(avg))
                return(queue, avg)
            print("Queue is not full yet!")
            return(queue, 0.0)

        def compute(point_cloud, x, y):
            err, point_cloud_value = point_cloud.get_value(x, y)

            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])

            if not np.isnan(distance) and not np.isinf(distance):
                return distance
            else:
                return 0
        
        def determine_slope(distances):
            # Check to see i the distances exist.
            if distances:
                line = []
                line = [ele - distances[-1] for ele in distances]
                slope = line[-1] - line[0]
            else:
                print('Unable to obtain distance measurement.')
                slope = 0

            return slope

        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        avg = []
        total = 0
        for i in range(3,7):
            for j in range(i*128,i*128+128):
                for k in range(340, 350):
                    total += compute(point_cloud, j, k)
            
            avg.append(total/(128*20))
            avg = [round(i,2) for i in avg]
            total = 0

        #print('total: ',avg)
        #print('avg_dist: ', sum(avg)/len(avg))
        #print('slopeself.: ', determine_slope(avg))
        
        distance_avg = sum(avg)/len(avg)
        slope = determine_slope(avg)

        self.distance_queue.append(distance_avg)
        self.distance_queue, distance_avg = moving_filter(self.distance_queue, self.filter_size)
        self.slope_queue.append(slope)
        self.slope_queue, slope_avg = moving_filter(self.slope_queue, self.filter_size)

        msg = Float32MultiArray()
        msg.data = [distance_avg, slope_avg]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    zed.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()