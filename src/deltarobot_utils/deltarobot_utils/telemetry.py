#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from deltarobot_interfaces.msg import JointPositionTelemetry

import numpy as np
import matplotlib.pyplot as plt



class Telemetry(Node):

    def __init__(self):
        super().__init__('telemetry_node')

        self.sub = self.create_subscription(
            JointPositionTelemetry,
            'joint_position_telemetry',
            self.telemetry_callback,
            100)
        self.sub

        return


    def telemetry_callback(self, joint_position_msg):
        ## unpack the msg
        t = joint_position_msg.t

        if t == 0:
            self.plot_index = 0
            self.q_carriage_1_array = np.empty((2,1000))
            self.q_carriage_2_array = np.empty((2,1000))
            self.q_carriage_3_array = np.empty((2,1000))
        
        elif t == -1.0:
            self.plot_datas()
            return

        q1_current = joint_position_msg.q[0]
        q2_current = joint_position_msg.q[1]
        q3_current = joint_position_msg.q[2]

        self.q_carriage_1_array[0, self.plot_index] = t
        self.q_carriage_1_array[1, self.plot_index] = q1_current

        self.q_carriage_2_array[0, self.plot_index] = t
        self.q_carriage_2_array[1, self.plot_index] = q2_current

        self.q_carriage_3_array[0, self.plot_index] = t
        self.q_carriage_3_array[1, self.plot_index] = q3_current        

        self.plot_index += 1
        return

    def plot_datas(self):
        # initialize plot
        fig1 = plt.figure()
        plot_pos = fig1.add_subplot(111)
        plt.legend()
        plt.title("joint position")
        plt.xlabel("time [ s ]")
        plt.ylabel("joint position [ mm ]")

        fig2 = plt.figure()
        plot_vel = fig2.add_subplot(111)
        plt.legend()
        plt.title("joint velocity")
        plt.xlabel("time [ s ]")
        plt.ylabel("joint velocity [ mm/s ]")

        fig3 = plt.figure()
        plot_acc = fig3.add_subplot(111)
        plt.legend()
        plt.title("joint acceleration")
        plt.xlabel("time [ s ]")
        plt.ylabel("joint acceleration [ mm/s2 ]")


        # plot position
        plot_pos.plot(self.q_carriage_1_array[0, 0:self.plot_index], self.q_carriage_1_array[1, 0:self.plot_index], "r", label="joint 1")
        plot_pos.plot(self.q_carriage_2_array[0, 0:self.plot_index], self.q_carriage_2_array[1, 0:self.plot_index], "g", label="joint 2")
        plot_pos.plot(self.q_carriage_3_array[0, 0:self.plot_index], self.q_carriage_3_array[1, 0:self.plot_index], "b", label="joint 3")
        
        # compute and plot velocity
        dt_q_carriage_1_array = np.gradient(self.q_carriage_1_array[1, 0:self.plot_index], self.q_carriage_1_array[0, 0:self.plot_index])
        dt_q_carriage_2_array = np.gradient(self.q_carriage_2_array[1, 0:self.plot_index], self.q_carriage_2_array[0, 0:self.plot_index])
        dt_q_carriage_3_array = np.gradient(self.q_carriage_3_array[1, 0:self.plot_index], self.q_carriage_3_array[0, 0:self.plot_index])

        plot_vel.plot(self.q_carriage_1_array[0, 0:self.plot_index], dt_q_carriage_1_array[0:self.plot_index], "r", label="dt joint 1")
        plot_vel.plot(self.q_carriage_2_array[0, 0:self.plot_index], dt_q_carriage_2_array[0:self.plot_index], "g", label="dt joint 2")
        plot_vel.plot(self.q_carriage_3_array[0, 0:self.plot_index], dt_q_carriage_3_array[0:self.plot_index], "b", label="dt joint 3")
        
        # compute and plot velocity
        ddt_q_carriage_1_array = np.gradient(dt_q_carriage_1_array[0:self.plot_index], self.q_carriage_1_array[0, 0:self.plot_index])
        ddt_q_carriage_2_array = np.gradient(dt_q_carriage_2_array[0:self.plot_index], self.q_carriage_2_array[0, 0:self.plot_index])
        ddt_q_carriage_3_array = np.gradient(dt_q_carriage_3_array[0:self.plot_index], self.q_carriage_3_array[0, 0:self.plot_index])

        plot_acc.plot(self.q_carriage_1_array[0, 0:self.plot_index], ddt_q_carriage_1_array[0:self.plot_index], "r", label="dt joint 1")
        plot_acc.plot(self.q_carriage_2_array[0, 0:self.plot_index], ddt_q_carriage_2_array[0:self.plot_index], "g", label="dt joint 2")
        plot_acc.plot(self.q_carriage_3_array[0, 0:self.plot_index], ddt_q_carriage_3_array[0:self.plot_index], "b", label="dt joint 3")
        
        plt.show()
        return
    


def main(args=None):
    rclpy.init(args=args)

    tl_node = Telemetry()

    rclpy.spin(tl_node)

    tl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()