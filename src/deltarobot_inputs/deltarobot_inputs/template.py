#!/usr/bin/env python3

'''

    Using this template:
        1. copy this file and rename it
            e.g. "my_algorithm.py"
        2. double click on this word --> "template", press ctrl+shift+F2 and rename it with snake convention.
            e.g. "my_algorithm"
        3. double click on this word --> "Template_", press ctrl+shift+F2 and rename it with capital letter.
            e.g. "MyAlgorithm"

'''


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import Bool
from std_msgs.msg import Int32



class Template_(Node):

    def __init__(self):

        super().__init__('template_node')

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
        self.task_queue_len__sub = self.create_subscription(
            Int32,
            'task_queue_len',
            self.task_queue_len__callback,
            1)
        
        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#


        return

    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def task_queue_len__callback(self):
        
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
        return    
    
    '''
        status = True,  go to home.
    '''
    def input_cmds__homing__publish(self, status):
        msg = Bool()
        msg.data = status
        self.input_cmds__homing__pub.publish(msg)
        return

    '''
        status = True,  the gripper is closed.
        status = False, the gripper is open.
    '''
    def input_cmds__gripper__em__publish(self, status):
        msg = Bool()
        msg.data = status
        self.input_cmds__gripper__em__pub.publish(msg)
        return



    ###################################################################################
    #                                                                                 #
    #                             ALGORITHM FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def my_algorithm_template(self):
        ## get the data

        ## do something

        ## publish the data
        
        return



def main(args=None):
    rclpy.init(args=args)

    template_node = Template_()

    rclpy.spin(template_node)

    template_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
