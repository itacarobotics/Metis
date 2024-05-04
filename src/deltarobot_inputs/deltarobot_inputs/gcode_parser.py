#!/usr/bin/env python3

'''

    Using this gcode_parser:
        1. copy this file and rename it
            e.g. "my_algorithm.py"
        2. double click on this word --> "GcodeParser", press ctrl+shift+F2 and rename it with capital letter.
            e.g. "MyAlgorithm"
        3. double click on this word --> "gcode_parser", press ctrl+shift+F2 and rename it with snake convention.
            e.g. "my_algorithm"

'''


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import Bool
from std_msgs.msg import Int32

from deltarobot import configuration as conf
from os.path import join



class GcodeParser(Node):

    def __init__(self):

        super().__init__('gcode_parser_node')

        #**********************************************************#
        #                     define publishers                    #
        #**********************************************************#

        # publish input commands
        self.input_cmds__move__task_space__ptp__pub = self.create_publisher(
            TrajectoryTask,
            'input_cmds/move/task_space/ptp',
            1)
        
        self.input_cmds__gripper__em__pub = self.create_publisher(
            Bool,
            'input_cmds/gripper/em',
            1)
        
        self.input_cmds__homing__pub = self.create_publisher(
            Bool,
            'input_cmds/homing',
            1)


        #**********************************************************#
        #                     define subscribers                   #
        #**********************************************************#

        
        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#
        

        return


    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def input_cmds__move__task_space__ptp__publish(self, x, y, z, t):
        msg = TrajectoryTask()

        msg.pos_end.x       = float(x)
        msg.pos_end.y       = float(y)
        msg.pos_end.z       = float(z)
        msg.time_total      = float(t)

        # Publish task
        self.input_cmds__move__task_space__ptp__pub.publish(msg)
        self.get_logger().info(f"move: {x}, {y}, {z}, {t}")
        return    
    
    '''
        status = True,  go to home.
    '''
    def input_cmds__homing__publish(self, status):
        msg = Bool()
        msg.data = status
        self.input_cmds__homing__pub.publish(msg)
        self.get_logger().info("homing")
        return

    '''
        status = True,  the gripper is closed.
        status = False, the gripper is open.
    '''
    def input_cmds__gripper__em__publish(self, status):
        msg = Bool()
        msg.data = status
        self.input_cmds__gripper__em__pub.publish(msg)
        self.get_logger().info(f"gripper: {status}")
        return



    ###################################################################################
    #                                                                                 #
    #                             ALGORITHM FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def parse_file(self, file_path):
        ## get the data
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    # Remove leading and trailing whitespaces
                    line = line.strip()
                    # Ignore empty lines and comments
                    if line and not line.startswith(";"):
                        line = line.split(";")[0]
                        self.parse_line(line)

        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} not found.")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        return



    def parse_line(self, line):
        # Split the line by whitespace to separate the G-code command and its arguments
        parts = line.split()
        # Extract the G-code command
        gcode_command = parts[0]

        # If it's a G28 command
        if gcode_command == "G28":
            self.input_cmds__homing__publish(True)
        
        # If it's a G0 command
        elif gcode_command == "G0":
            x = parts[1][1:]
            y = parts[2][1:]
            z = parts[3][1:]
            t = parts[4][1:]
            # Call the rapid move function
            self.input_cmds__move__task_space__ptp__publish(x, y, z, t)  # Assuming the last argument is always -1
        
        # If it's a G1 command
        elif gcode_command == "M5":
            # Check if it's turning the gripper on/off
            if "ON" in parts:
                self.input_cmds__gripper__em__publish(False)
            elif "OFF" in parts:
                self.input_cmds__gripper__em__publish(True)

        else:
            self.get_logger().error(f"Unknown G-code command: {gcode_command}")

        return



def main(args=None):
    rclpy.init(args=args)

    gcode_parser_node = GcodeParser()

    # file name
    file_name = "test1.gcode"
    file_path = join(conf.ws_path, "src/deltarobot_inputs/assets/gcode")
    file_path = join(file_path, file_name)
    
    # parse gcode file
    gcode_parser_node.parse_file(file_path)
    
    rclpy.spin_once(gcode_parser_node)

    gcode_parser_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
