#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String

import inputs
import numpy as np
import threading



class GamePadController(Node):

    def __init__(self):
        """
        Initializes the GamePadController node.
        """
        super().__init__('gamepad_controller_node')

        # Create publisher for trajectory task input
        self.trajectory_task_input_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task_input',
            1)
        
        # Set up timer to periodically send trajectory task
        timer_period = 0.1  # seconds
        self.publish_task_timer = self.create_timer(
            timer_period, 
            self.publish_task_timer_callback)
        
        # Subscribe to robot state topic
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = False

        # Get the gamepad device
        self.gamepad = inputs.devices.gamepads[0]
        self.x = 0                  # [ mm ]
        self.y = 0                  # [ mm ]
        self.z = 0                  # [ mm ]
        self.velocity = 50          # [ mm/s ]
        self.delta_time = 0.15      # [ s ]
        self.MAX_VELOCITY = 250     # [ mm/s ]
        self.MIN_VELOCITY = 10      # [ mm/s ]

        return
    

    def publish_task_timer_callback(self):
        """
        Timer callback function to send trajectory task based on gamepad input.
        """

        # If robot is in idle, it can move
        if self.pub_task_lock == False:
            if self.x == 0 and self.y == 0 and self.z == 0:
                return
            
            # Create trajectory task message
            trajectory_task_msg = TrajectoryTask()
            trajectory_task_msg.pos_end.x = float(self.x)
            trajectory_task_msg.pos_end.y = float(self.y)
            trajectory_task_msg.pos_end.z = float(self.z)
            trajectory_task_msg.task_time = float(self.delta_time)
            trajectory_task_msg.task_type.data = str(conf.P2P_DIRECT_TRAJECTORY)
            trajectory_task_msg.is_trajectory_absolute_coordinates = False

            # Publish trajectory task message
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
            # set a lock to publish once
            self.pub_task_lock = True
        return
    

    def robot_state_callback(self, robot_state_msg):
        """
        Callback function for receiving robot state.

        Args:
            robot_state_msg (String): The received robot state message.
        """
        
        # robot is ready to receive new task
        if robot_state_msg.data == conf.ROBOT_STATE_IDLE:
            self.pub_task_lock = False
        else:
            self.pub_task_lock = True

        return



    # ******************************** GAMEPAD thread ***********************************

    def gamepad_loop(self):

        while True:
            x, y, z = self.read_gamepad_state()
            self.update_gamepad_commands(x, y, z)
        # never ends


    def read_gamepad_state(self):      
        events = self.gamepad._do_iter(timeout=0.0002)

        # do not update gamepad states if no data available        
        if events is None:
            return None, None, None

        x_input = None
        y_input = None
        z_input = None
        sens_threshold = 0.2

        for event in events:
            ## get value X coordinate
            # joy
            if event.code == 'ABS_X':
                # normalized value [-1, 1]
                x_input = (event.state-127.5)/127.5
                if abs(x_input) < sens_threshold:
                    x_input = 0

            # hat
            elif event.code == 'ABS_HAT0X':
                x_input = event.state*self.velocity*self.delta_time
                y_input = 0
                z_input = 0

            ## get value Y coordinate
            # joy
            elif event.code == 'ABS_Y':
                # normalized value [-1, 1]
                y_input = -(event.state-127.5)/127.5
                if abs(y_input) < sens_threshold:
                    y_input = 0
            
            # hat
            elif event.code == 'ABS_HAT0Y':
                x_input = 0
                y_input = event.state*self.velocity*self.delta_time
                z_input = 0


            ## get value Zs coordinate
            elif  event.code == 'ABS_RZ':
                # normalized value [-1, 1]
                z_input = -(event.state-127.5)/127.5
                if abs(z_input) < sens_threshold:
                    z_input = 0

            ## set velocity
            # increase velocity
            elif  event.code == 'BTN_TR':
                self.velocity += 25
                self.velocity = min(self.velocity, self.MAX_VELOCITY)
                
            # decrease velocity
            elif  event.code == 'BTN_TL':
                self.velocity -= 25
                self.velocity = max(self.velocity, self.MIN_VELOCITY)

            ## control gripper
                # A -> "BTN_SOUTH"
                # B -> "BTN_WEST"
                # X -> "BTN_NORTH"
                # Y -> "BTN_EAST"

        return x_input, y_input, z_input
    
    def update_gamepad_commands(self, x_input, y_input, z_input):
        # normalize
        if x_input is None:
            x_norm = 0
        else:
            x_norm = x_input

        if y_input is None:
            y_norm = 0
        else:
            y_norm = y_input

        if z_input is None:
            z_norm = 0
        else:
            z_norm = z_input

        coordinates_norm = np.linalg.norm([x_norm, y_norm, z_norm])
        
        if coordinates_norm == 0:
            return
        

        # update states
        if x_input is not None:
            self.x = (x_input/coordinates_norm) * self.velocity*self.delta_time
        if y_input is not None:
            self.y = (y_input/coordinates_norm) * self.velocity*self.delta_time
        if z_input is not None:
            self.z = (z_input/coordinates_norm) * self.velocity*self.delta_time
        
        return
    
    # ******************************** GAMEPAD thread ***********************************




def main(args=None):
    """
    Main function.
    """
    rclpy.init(args=args)

    gp_node = GamePadController()

    # Create a thread for running the Tkinter main loop
    gp_thread = threading.Thread(target=gp_node.gamepad_loop)
    gp_thread.daemon = True  # Make the thread a daemon so it terminates when the main thread terminates
    gp_thread.start()

    rclpy.spin(gp_node)

    gp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
