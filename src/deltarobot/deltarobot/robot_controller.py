#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from deltarobot.delta_robot import DeltaRobot

from deltarobot import configuration as conf

from std_msgs.msg import String
from std_msgs.msg import Bool

from deltarobot_interfaces.msg import TrajectoryTask

from micro_custom_messages.msg import JointTrajectory
from micro_custom_messages.msg import JointTrajectoryArray

import numpy as np


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')
        
        #**********************************************************#
        #                     define publishers                    #
        #**********************************************************#
        
        ## publish robot commands
        self.robot_cmds__move__joint_trajectory__pub = self.create_publisher(
            JointTrajectoryArray,
            'robot_cmds/move/joint_trajectory',
            1)
        
        self.robot_cmds__gripper__em__pub = self.create_publisher(
            Bool,
            'robot_cmds/gripper/em',
            1)
        
        self.robot_cmds__homing__pub = self.create_publisher(
            Bool,
            'robot_cmds/homing',
            1)
        
        ## publish robot state
        self.robot_state__pub = self.create_publisher(
            String,
            'robot_state',
            1)


        #**********************************************************#
        #                     define subscribers                   #
        #**********************************************************#

        ## subscribe to /input_cmds
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

        ## subscribe to /robot_state topic
        self.robot_state__sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state__callback,
            1)

        ## subscribe to /feedback topic
        self.feedback__task_ack__sub = self.create_subscription(
            Bool,
            'feedback/task_ack',
            self.robot_feedback__ack__callback,
            1)


        ## init robot
        self.robot = DeltaRobot(conf)

        return


    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def input_cmds__move__task_space__ptp__callback(self, msg_in):
        
        ## unpack msg
        pos_end = np.empty(3)
        pos_end[0] = msg_in.pos_end.x
        pos_end[1] = msg_in.pos_end.y
        pos_end[2] = msg_in.pos_end.z
        time_total = msg_in.time_total

        ## generate joint trajectory
        joint_trajectory = self.robot.generate_joint_trajectory__task_space__ptp(pos_end, time_total)

        ## joint trajectory not valid
        if joint_trajectory is None:
            self.get_logger().error("Trajectory is not valid!")
            self.update_robot_state(conf.ROBOT_STATE_ERROR)
            return

        ## publish message
        self.robot_cmds__move__joint_trajectory__publish(joint_trajectory)
        self.update_robot_state(conf.ROBOT_STATE_RUN)
        return
    
    def input_cmds__gripper__em__callback(self, msg):
        self.robot_cmds__gripper__em__pub.publish(msg)
        self.update_robot_state(conf.ROBOT_STATE_RUN)        
        return
    
    def input_cmds__homing__callback(self, msg):
        self.robot_cmds__homing__pub.publish(msg)
        self.robot.update_robot_position(conf.pos_home)
        self.update_robot_state(conf.ROBOT_STATE_RUN)
        return


    def robot_state__callback(self, msg):
        ## override robot state
        self.robot.update_robot_state(msg.data)
        return
    

    def robot_feedback__ack__callback(self, msg_in):
        # The task has been successful
        if msg_in.data == True:
            # Update current positions
            self.robot.update_robot_position()

            if self.robot.get_robot_state() == conf.ROBOT_STATE_RUN:
                # Update robot state
                self.update_robot_state(conf.ROBOT_STATE_IDLE)
            else:
                # Do not override stop or error states
                pass

        else:
            # raise error    
            self.update_robot_state(conf.ROBOT_STATE_ERROR)
        
        return
    

    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def robot_cmds__move__joint_trajectory_viz__publish(self, np_array):
        return

    def robot_cmds__move__joint_trajectory__publish(self, joint_trajectory):
        msg_out = JointTrajectoryArray()
        msg_out.array_size = len(joint_trajectory)

        for joint_via_point in joint_trajectory:
            joint_via_point_msg = JointTrajectory()

            joint_via_point_msg.q1              = float(joint_via_point[0])
            joint_via_point_msg.q2              = float(joint_via_point[1])
            joint_via_point_msg.q3              = float(joint_via_point[2])

            joint_via_point_msg.t_via_point     = float(joint_via_point[3])

            msg_out.via_points.append(joint_via_point_msg)

        ## publish joint trajectory
        self.robot_cmds__move__joint_trajectory__pub.publish(msg_out)
        return
    
    def robot_state__publish(self, state):
        # Publish robot state
        msg_out = String()
        msg_out.data = state
        self.robot_state__pub.publish(msg_out)
        return


    ###################################################################################
    #                                                                                 #
    #                                UTILS FUNCTIONS                                  #
    #                                                                                 #
    ###################################################################################

    def update_robot_state(self, state):
        self.robot.update_robot_state(state)
        self.robot_state__publish(state)
        return






###################################################################################
#                                                                                 #
#                                      MAIN                                       #
#                                                                                 #
###################################################################################

def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()