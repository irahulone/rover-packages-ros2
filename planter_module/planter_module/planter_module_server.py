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

import time


class PlanterActionServer(Node):

    def __init__(self):
        self.serial = Serial("/dev/ttyACM0")
        self.serial.set()
        super().__init__('planter_action_server')
        self._action_server = ActionServer(
            self,
            planter,
            'planter_action',
            self.execute_callback)

        #conver belt shit
        conv_motor_1 = MotorNode("1", "/dev/ttyACM1")
        rclpy.init(None)
        try:
            rclpy.spin(conv_motor_1)
        except KeyboardInterrupt:
            conv_motor_1.get_logger().info('Shutting down node...')
            conv_motor_1.destroy_node()
            rclpy.shutdown()
        # conv_motor_2 = MotorNode("2", "/dev/ttyACM1")
        # conv_motor_3 = MotorNode("3", "/dev/ttyACM1")

        self.ref_coords1 = self.create_publisher(NavSatFix, '/r1/ref_coordinate1', 5)
        self.ref_coords2 = self.create_publisher(NavSatFix, '/r1/ref_coordinate2', 5)
        self.motor_1_speed = self.create_publisher(Int16, 'motor_1/set_speed', 5)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.phase_1()
        self.phase_2()

        goal_handle.succeed()

        result = planter.Result()
        result.current_position = 1
        return result

    def stage_1(self):
        self.serial.drillMotor("forward")
        self.serial.linActMotor("forward")

        recieved_serial = self.serial.getOutput()
        data = self.parse_output(recieved_serial)

        while data["ActuatorDistance"] > 90:
            recieved_serial = self.serial.getOutput()
            data = self.parse_output(recieved_serial)

        self.serial.drillMotor("backward")
        self.serial.linActMotor("backward")

        prevData = data

        time.sleep(1)

        recieved_serial = self.serial.getOutput()
        data = self.parse_output(recieved_serial)

        while abs(data["ActuatorDistance"] - prevData["ActuatorDistance"]) > 3: #rudimentary value as threshold
            recieved_serial = self.serial.getOutput()
            data = self.parse_output(recieved_serial)

        self.serial.drillMotor("stop")
        self.serial.linActMotor("stop")

    def parse_output(data):
        # Decode the byte data to a string
        decoded_data = data.decode('utf-8')
        
        # Split the data into lines
        lines = decoded_data.splitlines()
        
        # Initialize variables for parsing
        parsed_data = []
        capturing = False

        # Parse the lines
        for line in lines:
            line = line.strip()  # Remove extra whitespace
            
            if line == "START":
                capturing = True
                parsed_data = []  # Clear data buffer for a new block
            elif line == "END":
                capturing = False
                # Return the parsed block of data
                return parsed_data
            elif capturing:
                if ":" in line:  # Parse key-value pairs
                    key, value = line.split(":", 1)
                    parsed_data.append((key.strip(), value.strip()))
                else:
                    parsed_data.append((line.strip(), None))  # Handle single-line entries
        return None  # Return None if no complete block was found



    def stage_2(self):
        #activate conv grav system

        speed = Int16(20)
        self.motor_1_speed.publish(speed)

        while(1):
            recieved_serial = self.serial.getOutput()
            data = self.parse_output(recieved_serial)

            if data["Blocking"] == True:
                break
        
        speed = Int16(0)
        self.motor_1_speed.publish(speed)

        return

    def getSensorData():

        return




def main(args=None):
    rclpy.init(args=args)

    planter_action_server = PlanterActionServer()

    rclpy.spin(planter_action_server)


if __name__ == '__main__':
    main()