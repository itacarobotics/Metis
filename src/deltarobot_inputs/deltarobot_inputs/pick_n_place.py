#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String
from std_msgs.msg import Bool

import time


class PicknPlace(Node):

    def __init__(self):

        super().__init__('pnp_node')


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

        ## subscribe to /robot_state topic
        self.robot_state__sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state__callback,
            1)
        
        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#
        # Set up timer to periodically send trajectory task
        timer_period = 0.05  # seconds
        self.publish_task_timer = self.create_timer(
            timer_period, 
            self.timer_callback)
        

        ## homing
        self.input_cmds__homing__publish()
        time.sleep(1)
        # self.input_cmds__move__task_space__ptp__publish(50, 100, -220, 6)


        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = True


        self.schedule_index = 0
        
        ## pattern movements
        self.x_goal         = 60
        self.y_goal         = 120
        self.z_goal         = -237
        self.z_retraction   = 40
  
        return
    

    def robot_state__callback(self, msg):
        if msg.data == conf.ROBOT_STATE_IDLE:
            self.pub_task_lock = False

        return


    # def timer_callback(self):

    #     if self.pub_task_lock == False:

    #         if self.schedule_index == 0:
    #             self.input_cmds__move__task_space__ptp__publish(-50, -100, -220, -1)

    #         elif self.schedule_index == 1:
    #             self.input_cmds__move__task_space__ptp__publish(50, 100, -220, -1)
    #             self.schedule_index = -1   

    #         self.schedule_index += 1
            
    #         ## set lock
    #         self.pub_task_lock = True

    #     return



    # def timer_callback(self):

    #     if self.pub_task_lock == False:

    #         if self.schedule_index == 0:
    #             self.input_cmds__move__task_space__ptp__publish(-150, 0, -225, 5)

    #         elif self.schedule_index == 1:
    #             self.input_cmds__move__task_space__ptp__publish(0, 0, -200, -1)
            
    #         elif self.schedule_index == 2:
    #             self.input_cmds__move__task_space__ptp__publish(150, 0, -225, -1)

    #         elif self.schedule_index == 3:
    #             self.input_cmds__move__task_space__ptp__publish(0, 0, -200, 5)
    #             self.schedule_index = -1   

    #         self.schedule_index += 1
            
    #         ## set lock
    #         self.pub_task_lock = True

    #     return



    # def timer_callback(self):

    #     if self.pub_task_lock == False:

    #         if self.schedule_index == 0:
    #             self.input_cmds__move__task_space__ptp__publish(self.x_goal, self.y_goal, self.z_goal+self.z_retraction, -1)

    #         elif self.schedule_index == 1:
    #             self.input_cmds__move__task_space__ptp__publish(self.x_goal, self.y_goal, self.z_goal, 1)

    #         elif self.schedule_index == 2:
    #             self.input_cmds__gripper__em__publish(False),   # closed

    #         elif self.schedule_index == 3:
    #             self.input_cmds__move__task_space__ptp__publish(-self.x_goal, -self.y_goal, self.z_goal+self.z_retraction, -1)

    #         elif self.schedule_index == 4:
    #             self.input_cmds__move__task_space__ptp__publish(-self.x_goal, -self.y_goal, self.z_goal, 1)

    #         elif self.schedule_index == 5:
    #             self.input_cmds__gripper__em__publish(True),    # open

    #         elif self.schedule_index == 6:
    #             self.input_cmds__move__task_space__ptp__publish(-self.x_goal, -self.y_goal, self.z_goal+self.z_retraction, 1)

    #         elif self.schedule_index == 7:
    #             self.input_cmds__move__task_space__ptp__publish(-self.x_goal, -self.y_goal, self.z_goal, 1)

    #         elif self.schedule_index == 8:
    #             self.input_cmds__gripper__em__publish(False),   # closed

    #         elif self.schedule_index == 9:
    #             self.input_cmds__move__task_space__ptp__publish(self.x_goal, self.y_goal, self.z_goal+self.z_retraction, -1)

    #         elif self.schedule_index == 10:
    #             self.input_cmds__move__task_space__ptp__publish(self.x_goal, self.y_goal, self.z_goal, 1)

    #         elif self.schedule_index == 11:
    #             self.input_cmds__gripper__em__publish(True),    # open
    #             self.schedule_index = -1

    #         self.schedule_index += 1

    #         ## set lock
    #         self.pub_task_lock = True

    #     return


    def timer_callback(self):

        if self.pub_task_lock == False:

            ## start
            if self.schedule_index == 0:
                self.input_cmds__move__task_space__ptp__publish(0, 0, -120, 3)

            ## movement 1
            elif self.schedule_index == 1:
                self.input_cmds__move__task_space__ptp__publish(50, -100, -220, -1)
            
            elif self.schedule_index == 2:
                self.input_cmds__move__task_space__ptp__publish(50, -100, -237, -1)

            elif self.schedule_index == 3:
                self.input_cmds__gripper__em__publish(False),   # closed

            elif self.schedule_index == 4:
                self.input_cmds__move__task_space__ptp__publish(0, 100, -170, -1)

            elif self.schedule_index == 5:
                self.input_cmds__gripper__em__publish(True),   # open

            ## movement 2
            elif self.schedule_index == 6:
                self.input_cmds__move__task_space__ptp__publish(0, -100, -220, -1)
            
            elif self.schedule_index == 7:
                self.input_cmds__move__task_space__ptp__publish(0, -100, -237, -1)

            elif self.schedule_index == 8:
                self.input_cmds__gripper__em__publish(False),   # closed

            elif self.schedule_index == 9:
                self.input_cmds__move__task_space__ptp__publish(0, 100, -170, -1)

            elif self.schedule_index == 10:
                self.input_cmds__gripper__em__publish(True),   # open


            ## movement 3
            elif self.schedule_index == 11:
                self.input_cmds__move__task_space__ptp__publish(-50, -100, -220, -1)
            
            elif self.schedule_index == 12:
                self.input_cmds__move__task_space__ptp__publish(-50, -100, -237, -1)

            elif self.schedule_index == 13:
                self.input_cmds__gripper__em__publish(False),   # closed

            elif self.schedule_index == 14:
                self.input_cmds__move__task_space__ptp__publish(0, 100, -170, -1)

            elif self.schedule_index == 15:
                self.input_cmds__gripper__em__publish(True),   # open

            ## end
            elif self.schedule_index == 16:
                self.input_cmds__move__task_space__ptp__publish(0, 0, -80, 3)


            self.schedule_index += 1

            ## set lock
            self.pub_task_lock = True

        return


    def input_cmds__move__task_space__ptp__publish(self, x, y, z, t):
        msg = TrajectoryTask()

        msg.pos_end.x       = float(x)
        msg.pos_end.y       = float(y)
        msg.pos_end.z       = float(z)
        msg.time_total      = float(t)

        # Publish task
        self.input_cmds__move__task_space__ptp__pub.publish(msg)
        return    
    
    def input_cmds__homing__publish(self):
        msg = Bool()
        msg.data = True
        self.input_cmds__homing__pub.publish(msg)
        return

    def input_cmds__gripper__em__publish(self, status):
        msg = Bool()
        msg.data = status
        self.input_cmds__gripper__em__pub.publish(msg)
        return




def main(args=None):
    rclpy.init(args=args)

    pnp_node = PicknPlace()

    rclpy.spin(pnp_node)

    pnp_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
