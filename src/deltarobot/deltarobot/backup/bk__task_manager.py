#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask

from micro_custom_messages.msg import TaskAck

from std_msgs.msg import String
from std_msgs.msg import Bool

from deltarobot import configuration as conf

import numpy as np
import time

class TaskManager(Node):

    def __init__(self):
        """
        Initializes the TaskManager node.
        """
        super().__init__('task_manager_node')

        ## Subscribe to the trajectory task input topic
        self.trajectory_task_input_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task_input',
            self.trajectory_task_input_callback,
            1)

        ## Publish trajectory task
        self.trajectory_task_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task',
            1)
        
        ## Subscribe to task acknowledgment topic
        self.task_ack_sub = self.create_subscription(
            TaskAck,
            'task_ack',
            self.task_ack_callback,
            1)

        ## Publish robot state
        self.robot_state_pub = self.create_publisher(
            String,
            'robot_state',
            1)
        
        ## Subscribe to robot state topic
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)
        
        self.homing_sub = self.create_subscription(
            Bool,
            'task_homing',
            self.update_homing_position_callback,
            1)


        # Initialize robot position
        self.pos_current = conf.configuration["trajectory"]["pos_home"]
        self.pos_current_volatile = conf.configuration["trajectory"]["pos_home"]
        # self.pos_current = np.empty(3)    # uncomment if limit switch
        # self.pos_current_volatile = np.empty(3)
        
        # Initialize robot state
        self.robot_state = conf.ROBOT_STATE_IDLE

        return


    def trajectory_task_input_callback(self, task_input_msg):
        """
        Callback function for receiving trajectory task input.
        
        Parameters:
            task_input_msg (TrajectoryTask): The received trajectory task input message.
        
        Returns:
            None
        """
        task_output_msg = TrajectoryTask()

        # Define starting position
        task_output_msg.pos_start.x = float(self.pos_current[0])
        task_output_msg.pos_start.y = float(self.pos_current[1])
        task_output_msg.pos_start.z = float(self.pos_current[2])

        # Define ending position
        if task_input_msg.is_trajectory_absolute_coordinates == True:
            task_output_msg.pos_end = task_input_msg.pos_end
        else:
            task_output_msg.pos_end.x = task_input_msg.pos_end.x + self.pos_current[0]
            task_output_msg.pos_end.y = task_input_msg.pos_end.y + self.pos_current[1]
            task_output_msg.pos_end.z = task_input_msg.pos_end.z + self.pos_current[2]

        # Define other parameters
        task_output_msg.task_time = task_input_msg.task_time
        task_output_msg.task_type.data = task_input_msg.task_type.data
        task_output_msg.is_trajectory_absolute_coordinates = task_input_msg.is_trajectory_absolute_coordinates


        # Publish task message
        self.trajectory_task_pub.publish(task_output_msg)

        # Store the future pos_current in a volatile variable
        # In case of nak, the pos_current will remain the same
        # In case of ack, the pos_current will be updated with pos_current_volatile
        self.pos_current_volatile[0] = task_output_msg.pos_end.x
        self.pos_current_volatile[1] = task_output_msg.pos_end.y
        self.pos_current_volatile[2] = task_output_msg.pos_end.z
        
        
        # Update robot state
        self.robot_state = conf.ROBOT_STATE_RUN
        # Publish robot state
        self.publish_robot_state(self.robot_state)

        return


    def task_ack_callback(self, task_ack_msg):
        """
        Callback function for receiving task acknowledgment.
        
        Parameters:
            task_ack_msg (TaskAck): The received task acknowledgment message.
        
        Returns:
            None
        """
        # The task has been successful
        if task_ack_msg.task_ack == True:
            # Update current positions
            self.pos_current = np.copy(self.pos_current_volatile)

            if self.robot_state == conf.ROBOT_STATE_RUN:
                # Update robot state
                self.robot_state = conf.ROBOT_STATE_IDLE
            else:
                # Do not override stop or error states
                pass

        elif task_ack_msg.task_ack == False:
            # Raise error
            self.robot_state = conf.ROBOT_STATE_ERROR

        
        # Publish robot state
        self.publish_robot_state(self.robot_state)
        return
    

    def robot_state_callback(self, robot_state_msg):
        """
        Callback function for receiving robot state.
        
        Parameters:
            robot_state_msg (String): The received robot state message.
        
        Returns:
            None
        """
        # Respond with the robot state
        if robot_state_msg.data == conf.ROBOT_STATE_REQUEST:
            self.publish_robot_state(self.robot_state)

        # Override robot state
        else:
            self.robot_state = robot_state_msg.data
        
        return


    def publish_robot_state(self, state):
        """
        Publishes the robot state.
        
        Parameters:
            state (str): The current state of the robot.
        
        Returns:
            None
        """
        # Publish robot state
        robot_state_output_msg = String()
        robot_state_output_msg.data = str(state)
        self.robot_state_pub.publish(robot_state_output_msg)
        return

    def update_homing_position_callback(self, homing_msg):
        if homing_msg.data:
            self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
            self.pos_current_volatile = conf.configuration["trajectory"]["pos_home"]     ## after home calibration

        return


def main(args=None):
    rclpy.init(args=args)

    tm_node = TaskManager()

    rclpy.spin(tm_node)

    tm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
