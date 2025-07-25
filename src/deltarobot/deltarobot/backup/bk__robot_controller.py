#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import JointTrajectory
from deltarobot_interfaces.msg import JointTrajectoryArray

from micro_custom_messages.msg import JointTrajectoryReduced
from micro_custom_messages.msg import JointTrajectoryReducedArray
from micro_custom_messages.msg import TaskAck


from deltarobot.inverse_geometry import InverseGeometry
from deltarobot.trajectory_generator import TrajectoryGenerator
from deltarobot import configuration as conf

import pinocchio as pin
import numpy as np

from os.path import join
import time


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.joint_trajectory_reduced_pub = self.create_publisher(
            JointTrajectoryReducedArray,
            'joint_trajectory_reduced',
            1)
        
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectoryArray,
            'joint_trajectory',
            1)
        
        self.task_ack_pub = self.create_publisher(
            TaskAck,
            'task_ack',
            1)
        
        self.trajectory_task_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.robot_controller_callback,
            1)
        


        ## import urdf file path
        package_path = conf.configuration["paths"]["package_path"]
        # chain 1
        urdf_file_name_chain_1 = "deltarobot_c1.urdf"
        urdf_file_path_chain_1 = join(join(package_path, "urdf"), urdf_file_name_chain_1)
        # chain 2
        urdf_file_name_chain_2 = "deltarobot_c2.urdf"
        urdf_file_path_chain_2 = join(join(package_path, "urdf"), urdf_file_name_chain_2)
        # chain 3
        urdf_file_name_chain_3 = "deltarobot_c3.urdf"
        urdf_file_path_chain_3 = join(join(package_path, "urdf"), urdf_file_name_chain_3)

        ## Load the urdf models
        # chain 1
        model_chain_1 = pin.buildModelFromUrdf(urdf_file_path_chain_1)
        data_chain_1 = model_chain_1.createData()
        # chain 2
        model_chain_2 = pin.buildModelFromUrdf(urdf_file_path_chain_2)
        data_chain_2 = model_chain_2.createData()
        # chain 3
        model_chain_3 = pin.buildModelFromUrdf(urdf_file_path_chain_3)
        data_chain_3 = model_chain_3.createData()

        # total number of joints 
        self.robot_nq = model_chain_1.nq + model_chain_2.nq + model_chain_3.nq

        ## initialize InverseGeometry object
        self.ig_chain_1 = InverseGeometry(model_chain_1, data_chain_1, 10)
        self.ig_chain_2 = InverseGeometry(model_chain_2, data_chain_2, 10)
        self.ig_chain_3 = InverseGeometry(model_chain_3, data_chain_3, 10)

        ## initialize trajectory generator
        self.trajectory_generator = TrajectoryGenerator()

        # define initial conditions
        self.pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration
        self.end_effector_radius = conf.configuration["physical"]["end_effector_radius"]

        # joint limits
        self.q_min = 0    # [ mm ]
        self.q_max = 500    # [ mm ]

        # initialize joint position
        self.q1 = np.zeros(3)
        self.q2 = np.zeros(3)
        self.q3 = np.zeros(3)

        return
    

    def robot_controller_callback(self, trajectory_task_msg):
        # time_start = time.time()
        # unpack message
        pos_start = np.array([trajectory_task_msg.pos_start.x,
                            trajectory_task_msg.pos_start.y,
                            trajectory_task_msg.pos_start.z])
        
        pos_end = np.array([trajectory_task_msg.pos_end.x,
                            trajectory_task_msg.pos_end.y,
                            trajectory_task_msg.pos_end.z])
        
        delta_t_input = trajectory_task_msg.task_time
        task_type = trajectory_task_msg.task_type.data
        

        ## generate trajectory task space
        if task_type == conf.PLACE_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__bezier(
                pos_start, pos_end, delta_t_input, task_type)
            is_joint_trajectory = False
        
        elif task_type == conf.PICK_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__bezier(
                pos_start, pos_end, delta_t_input, task_type)
            is_joint_trajectory = False
        
        elif task_type == conf.P2P_DIRECT_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__point_to_point__direct(
                pos_start, pos_end, delta_t_input)
            is_joint_trajectory = False
            
        elif task_type == conf.P2P_JOINT_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__point_to_point__direct(
                pos_start, pos_end, delta_t_input)
            is_joint_trajectory = True
        
        elif task_type == conf.P2P_CONTINUOUS_TRAJECTORY:
            task_space_trajectory_vector = self.trajectory_generator.generate_trajectory__task_space__point_to_point__continuous(
                pos_start, pos_end, delta_t_input)
            is_joint_trajectory = False 

        ## check for valid trajectory
        if task_space_trajectory_vector is None:
            # publish nak
            self.get_logger().error("Invalid trajectory: trajectory time is not feasable.")
            nak_msg  = TaskAck()
            nak_msg.task_ack = False
            self.task_ack_pub.publish(nak_msg)
            return

        ## inverse geometry
        joint_trajectory_vector = np.empty((len(task_space_trajectory_vector), self.robot_nq+1))
            
        # end effector offsets
        ee_1_offset = np.array([np.sin(0), np.cos(0), 0])*self.end_effector_radius
        ee_2_offset = np.array([-np.sin((2/3)*np.pi), np.cos((2/3)*np.pi), 0])*self.end_effector_radius
        ee_3_offset = np.array([-np.sin((4/3)*np.pi), np.cos((4/3)*np.pi), 0])*self.end_effector_radius

        for set_point_index, set_point in enumerate(task_space_trajectory_vector):
            pos_des = set_point[0:3]
            t_set_point = set_point[3]
            
            ## compute inverse geometry
            # chain 1
            pos_des_1 = pos_des + ee_1_offset
            self.q1 = self.ig_chain_1.compute_inverse_geometry(np.copy(self.q1), pos_des_1)

            # chain 2
            pos_des_2 = pos_des + ee_2_offset
            self.q2 = self.ig_chain_2.compute_inverse_geometry(np.copy(self.q2), pos_des_2)

            # chain 3
            pos_des_3 = pos_des + ee_3_offset
            self.q3 = self.ig_chain_3.compute_inverse_geometry(np.copy(self.q3), pos_des_3)

            # check for collisions
            if self.check_is_collision():
                # publish nak
                self.get_logger().error("Invalid position: joint out of range.")
                nak_msg  = TaskAck()
                nak_msg.task_ack = False
                self.task_ack_pub.publish(nak_msg)
                return

            joint_trajectory_vector[set_point_index, 0:self.robot_nq] = np.concatenate([
                self.q1, self.q2, self.q3
            ])
            joint_trajectory_vector[set_point_index, self.robot_nq] = t_set_point

        # generate joint velocity profiles
        if is_joint_trajectory:
            q_start = np.array([
                joint_trajectory_vector[0, 0],      # chain 1
                joint_trajectory_vector[0, 3],      # chain 2
                joint_trajectory_vector[0, 6]])     # chain 3
            q_end = np.array([
                joint_trajectory_vector[len(task_space_trajectory_vector)-1, 0],    # chain 1
                joint_trajectory_vector[len(task_space_trajectory_vector)-1, 3],    # chain 2
                joint_trajectory_vector[len(task_space_trajectory_vector)-1, 6]])   # chain 3

            # overwrite with joint velocity profiles
            joint_trajectory_vector = self.trajectory_generator.generate_trajectory_joint_space(
                q_start, q_end, t_set_point)
        else:
            # publish full message for viewer
            self.publish_joint_trajectory(joint_trajectory_vector)


        # publish reduced message for microcontroller
        self.publish_joint_trajectory_reduced(joint_trajectory_vector)


        # time_stop = time.time()
        # self.get_logger().info(f"computation time: {(time_stop-time_start)*1e3} milliseconds")
        return
    

    def check_is_collision(self):
        if self.q1[0] < self.q_min or self.q1[0] > self.q_max:
            return True
        elif self.q2[0] < self.q_min or self.q2[0] > self.q_max:
            return True
        elif self.q3[0] < self.q_min or self.q3[0] > self.q_max:
            return True

        return False
    

    def publish_joint_trajectory(self, joint_trajectory_vector):
        joint_trajectory_array_msg = JointTrajectoryArray()
        joint_trajectory_array_msg.array_size = int(len(joint_trajectory_vector))

        for joint_trajectory in joint_trajectory_vector:
            joint_trajectory_msg = JointTrajectory()

            joint_trajectory_msg.q1_1 = float(joint_trajectory[0])
            joint_trajectory_msg.q1_2 = float(joint_trajectory[1])
            joint_trajectory_msg.q1_3 = float(joint_trajectory[2])

            joint_trajectory_msg.q2_1 = float(joint_trajectory[3])
            joint_trajectory_msg.q2_2 = float(joint_trajectory[4])
            joint_trajectory_msg.q2_3 = float(joint_trajectory[5])

            joint_trajectory_msg.q3_1 = float(joint_trajectory[6])
            joint_trajectory_msg.q3_2 = float(joint_trajectory[7])
            joint_trajectory_msg.q3_3 = float(joint_trajectory[8])

            joint_trajectory_msg.t_set_point = float(joint_trajectory[9])

            joint_trajectory_array_msg.set_points.append(joint_trajectory_msg)

        self.joint_trajectory_pub.publish(joint_trajectory_array_msg)
        return
    

    def publish_joint_trajectory_reduced(self, joint_trajectory_vector):
        joint_trajectory_array_msg = JointTrajectoryReducedArray()
        joint_trajectory_array_msg.array_size = len(joint_trajectory_vector)
        nq = len(joint_trajectory_vector[0])-1

        for joint_trajectory in joint_trajectory_vector:
            joint_trajectory_msg = JointTrajectoryReduced()

            joint_trajectory_msg.q1 = float(joint_trajectory[0])
            joint_trajectory_msg.q2 = float(joint_trajectory[int(nq/3)])
            joint_trajectory_msg.q3 = float(joint_trajectory[int(nq*2/3)])
            joint_trajectory_msg.t_set_point = float(joint_trajectory[int(nq)])

            joint_trajectory_array_msg.set_points.append(joint_trajectory_msg)

        self.joint_trajectory_reduced_pub.publish(joint_trajectory_array_msg)
        return


def main(args=None):
    rclpy.init(args=args)

    rc_node = RobotController()

    rclpy.spin(rc_node)

    rc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()