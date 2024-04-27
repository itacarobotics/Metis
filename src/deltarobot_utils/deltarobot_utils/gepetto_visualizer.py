#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import JointTrajectoryArray

from micro_custom_messages.msg import TaskAck

from deltarobot import configuration as conf

import pinocchio as pin
from pinocchio import RobotWrapper
import numpy as np

from os.path import join
import time


class GepettoVisualizer(Node):

    def __init__(self):
        super().__init__('gepetto_visualizer_node')
       
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectoryArray,
            'joint_trajectory',
            self.display_callback,
            1)
        self.joint_trajectory_sub

        self.task_ack_pub = self.create_publisher(
            TaskAck,
            'task_ack',
            1)

        ## init robot model
        # import URDF files
        package_path = conf.configuration["paths"]["package_path"]
        urdf_file_name = "deltarobot.urdf"
        urdf_file_path = join(join(package_path, "urdf"), urdf_file_name)
        mesh_dir = join(package_path,"meshes")

        # Initialize the model
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        self.robot = RobotWrapper(model, collision_model, visual_model)

        # Initialize the VIEWER
        self.robot.initViewer(loadModel=True)
        self.robot.viz.displayVisuals(True)



    def display_callback(self, joint_trajectory_array_msg):
        q = np.empty(self.robot.nq)
        t_previous = 0

        for joint_trajectory_msg in joint_trajectory_array_msg.set_points:
            ## unpack the message
            # chain 1
            q[0] = joint_trajectory_msg.q1_1
            q[1] = joint_trajectory_msg.q1_2
            q[2] = joint_trajectory_msg.q1_3
            q[3] = joint_trajectory_msg.q1_2
            q[4] = joint_trajectory_msg.q1_3
            # chain 2
            q[5] = joint_trajectory_msg.q2_1
            q[6] = joint_trajectory_msg.q2_2
            q[7] = joint_trajectory_msg.q2_3
            q[8] = joint_trajectory_msg.q2_2
            q[9] = joint_trajectory_msg.q2_3
            # chain 3
            q[10] = joint_trajectory_msg.q3_1
            q[11] = joint_trajectory_msg.q3_2
            q[12] = joint_trajectory_msg.q3_3
            q[13] = joint_trajectory_msg.q3_2
            q[14] = joint_trajectory_msg.q3_3
        
            self.robot.viz.display(q)

            delta_t = joint_trajectory_msg.t_set_point - t_previous
            t_previous = joint_trajectory_msg.t_set_point
            time.sleep(delta_t)
        
        ## publish ack
        # task_ack_msg = TaskAck()
        # task_ack_msg.task_ack = True
        # self.task_ack_pub.publish(task_ack_msg)
        return




def main(args=None):
    rclpy.init(args=args)

    viz_node = GepettoVisualizer()

    rclpy.spin(viz_node)

    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()