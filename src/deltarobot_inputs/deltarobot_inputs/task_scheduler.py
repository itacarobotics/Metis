#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32


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

        
        self.task_queue_len__pub = self.create_publisher(
            Int32,
            'task_queue_len',
            1)

        #**********************************************************#
        #                     define subscribers                   #
        #**********************************************************#

        ## subscribe to /robot_state topic
        self.robot_state__sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state__callback,
            1)


        ## subscribe to /robot_cmds
        self.input_cmds__move__task_space__ptp__sub = self.create_subscription(
            TrajectoryTask,
            'input_cmds/move/task_space/ptp',
            self.input_cmds__move__task_space__ptp__callback,
            1)

        self.input_cmds__gripper__em__sub = self.create_subscription(
            Bool,
            'input_cmds/gripper/em',
            self.input_cmds__gripper__em__callback,
            1)
        
        self.input_cmds__homing__sub = self.create_subscription(
            Bool,
            'input_cmds/homing',
            self.input_cmds__homing__callback,
            1)
        
        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#
        # Set up timer to periodically send trajectory task
        timer_period = 0.001  # seconds
        self.publish_task_timer = self.create_timer(
            timer_period, 
            self.timer_callback)
        
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

    def input_cmds__move__task_space__ptp__callback(self, msg):
        task = Task(msg, "input_cmds__move__task_space__ptp")
        
        # do not add same task
        if len(self.task_queue_list) != 0 and task == self.task_queue_list[-1]:
            return
        
        self.task_queue_list.append(task)
        # self.new_task_from_queue__publish()
        return

    def input_cmds__gripper__em__callback(self, msg):
        task = Task(msg, "input_cmds__gripper__em")
        
        # do not add same task
        if len(self.task_queue_list) != 0 and task == self.task_queue_list[-1]:
            return

        self.task_queue_list.append(task)
        
        # self.new_task_from_queue__publish()
        return
    
    def input_cmds__homing__callback(self, msg):
        # remove lock
        self.pub_task_lock = False
        # clear queue list
        self.task_queue_list.clear()

        task = Task(msg, "input_cmds__homing")
        self.task_queue_list.append(task)
        
        self.get_logger().info(f"lock is {self.pub_task_lock} and queue length is {len(self.task_queue_list)}")
        
        # self.new_task_from_queue__publish()
        return


    '''
        If robot_state has changed and is ready to handle new tasks, removes publishing lock.
    '''
    def robot_state__callback(self, msg):
        if msg.data == conf.ROBOT_STATE_IDLE:
            self.pub_task_lock = False
            # self.new_task_from_queue__publish()
        else:
            self.pub_task_lock = True

        return


    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def new_task_from_queue__publish(self):
        if self.pub_task_lock == False and len(self.task_queue_list) != 0:
            self.get_logger().info(f"new message queue length = {len(self.task_queue_list)}")
            ## publish message
            msg = self.task_queue_list[0].msg

            if self.task_queue_list[0].task_type == "input_cmds__move__task_space__ptp":
                self.robot_cmds__move__task_space__ptp__pub.publish(msg)

            elif self.task_queue_list[0].task_type == "input_cmds__gripper__em":
                self.robot_cmds__gripper__em__pub.publish(msg)

            elif self.task_queue_list[0].task_type == "input_cmds__homing":
                self.robot_cmds__homing__pub.publish(msg)

            # remove task from list
            self.task_queue_list.pop(0)
            # update new list length
            self.task_queue_len__publish(len(self.task_queue_list))
            
            # avoid new tasks beeing published
            self.pub_task_lock = True

        return


    def task_queue_len__publish(self, len):
        msg = Int32()
        msg.data = len
        self.task_queue_len__pub.publish(msg)
        return

    def timer_callback(self):
        self.new_task_from_queue__publish()
        return




def main(args=None):
    rclpy.init(args=args)

    task_scheduler_node = TaskScheduler()

    rclpy.spin(task_scheduler_node)

    task_scheduler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
