#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16


class Task():
    def __init__(self, msg, task_type):
        self.msg = msg
        self.task_type = task_type
        return


class TaskScheduler(Node):

    def __init__(self):

        super().__init__('task_scheduler_node')

        #**********************************************************#
        #                     define publishers                    #
        #**********************************************************#
        
        # publish input commands
        self.robot_cmds__move__task_space__ptp__pub = self.create_publisher(
            TrajectoryTask,
            'robot_cmds/move/task_space/ptp',
            1)

        self.robot_cmds__gripper__em__pub = self.create_publisher(
            Bool,
            'robot_cmds/gripper/em',
            1)
        
        self.robot_cmds__homing__pub = self.create_publisher(
            Bool,
            'robot_cmds/homing',
            1)

        
        self.task_scheduler_queue_len__pub = self.create_publisher(
            Int16,
            'task_scheduler_queue_len',
            10)

        #**********************************************************#
        #                     define subscribers                   #
        #**********************************************************#

        ## subscribe to /robot_state topic
        self.robot_state__sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state__callback,
            1)


        ## subscribe to /input_gcode_cmds
        self.input_gcode_cmds__pub__sub = self.create_subscription(
            String,
            'input_gcode_cmds',
            self.input_gcode_cmds__pub__callback,
            1)


        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#
        # Set up timer to periodically send trajectory task
        timer_period = 0.001  # seconds
        self.publish_task_timer = self.create_timer(
            timer_period, 
            self.task_publisher__timer_callback)

        # task queue
        self.task_queue_list = []

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = True

        return


    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def input_gcode_cmds__pub__callback(self, msg):
        if msg.data == "G28":   # cmd is homing
            # remove lock
            self.pub_task_lock = False
            # clear queue list
            self.task_queue_list.clear()

        self.task_queue_list.append(msg.data)
        return


    '''
        If robot_state has changed and is ready to handle new tasks, removes publishing lock.
    '''
    def robot_state__callback(self, msg):
        if msg.data == conf.ROBOT_STATE_IDLE:
            self.pub_task_lock = False
        else:
            self.pub_task_lock = True

        return


    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def robot_cmds__move__task_space__ptp__publish(self, task):
        msg = TrajectoryTask()

        task_parts = task.split()

        # Assign the extracted values
        msg.pos_end.x   = float(task_parts[1][1:])
        msg.pos_end.y   = float(task_parts[2][1:])
        msg.pos_end.z   = float(task_parts[3][1:])
        msg.time_total  = float(task_parts[4][1:])

        # publish msg
        self.robot_cmds__move__task_space__ptp__pub.publish(msg)
        return
    

    def robot_cmds__gripper__em__publish(self, data_in):
        msg = Bool()
        msg.data = data_in
        
        self.robot_cmds__gripper__em__pub.publish(msg)
        return
    

    def robot_cmds__homing__publish(self):
        msg = Bool()
        msg.data = True
        self.robot_cmds__homing__pub.publish(msg)       
        return


    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def task_publisher__timer_callback(self):
        # check if robot is ready to handle new task
        if self.pub_task_lock == False and len(self.task_queue_list) > 0:
            ## publish message
            task = self.task_queue_list[0]
            cmd_type = task[0:3]

            if cmd_type == "G01":
                self.robot_cmds__move__task_space__ptp__publish(task)
            elif cmd_type == "G28":
                self.robot_cmds__homing__publish()
            elif cmd_type == "M05":
                self.robot_cmds__gripper__em__publish(True)
            elif cmd_type == "M03":
                self.robot_cmds__gripper__em__publish(False)
            else:
                self.get_logger().error(f"{task}: INVALID CMD!")
            
            # remove task from list
            self.task_queue_list.pop(0)

            # publish task queue length
            msg = Int16()
            msg.data = len(self.task_queue_list)
            self.task_scheduler_queue_len__pub.publish(msg)

            # avoid new tasks beeing published
            self.pub_task_lock = True

        return




def main(args=None):
    rclpy.init(args=args)

    task_scheduler_node = TaskScheduler()

    rclpy.spin(task_scheduler_node)

    task_scheduler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
