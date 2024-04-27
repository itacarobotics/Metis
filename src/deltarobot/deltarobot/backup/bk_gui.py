#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import ttk
import threading

from deltarobot import configuration as conf

from deltarobot_interfaces.msg import TrajectoryTask
from std_msgs.msg import String
from std_msgs.msg import Bool



class GUI(Node):

    def __init__(self):
        """
        Initializes the GUI node.
        """
        super().__init__('gui_node')

        # Publisher for trajectory task input
        self.trajectory_task_input_pub = self.create_publisher(
            TrajectoryTask,
            'trajectory_task_input',
            1)
        
        ## Subscribe to robot state topic
        self.robot_state_sub = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            1)
        
        # Publisher for robot state
        self.robot_state_pub = self.create_publisher(
            String,
            'robot_state',
            1)
        
        # Publisher for homing
        self.homing_pub = self.create_publisher(
            Bool,
            'task_homing',
            1)
        

        ## Subscribe to trajectory task topic
        self.robot_state_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.update_display_position_callback,
            1)
        

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = False

        return
    

    # *********************************** GUI thread ***********************************
    
    def relative_to_assets(self, path):
        """
        Generates the absolute path to the assets folder.
        
        Parameters:
            path (str): The path to the asset.
        
        Returns:
            str: The absolute path to the asset.
        """
        ASSETS_PATH = conf.configuration["paths"]["gui_assets_path"]
        return str(ASSETS_PATH + path)
    
    def init_gui(self):
        """
        Initializes the graphical user interface.
        """
        # GUI parameters
        bg_color = "#2B3499"
        fg_color = "#FFFFFF"
        entry_font = ("Helvetica", 16, "bold")

        # Initialize main window
        self.window = tk.Tk()
        self.window.title("GUI robot controller")
        self.window.geometry("640x980")
        self.window.resizable(False, False)

        # Create canvas for background image
        self.canvas = tk.Canvas(
            self.window,
            height = 980,
            width = 640,
            bd = 0,
            highlightthickness = 0,
        )
        self.canvas.place(x = 0, y = 0)

        # Load and display background image
        self.image_bg = tk.PhotoImage(
            file=self.relative_to_assets("bg.png"))
        self.canvas.create_image(
            320.0,
            490.0,
            image=self.image_bg
        )

        # STOP button
        self.button_stop_img = tk.PhotoImage(
            file=self.relative_to_assets("button_stop.png"))
        self.button_stop = tk.Button(
            image=self.button_stop_img,
            borderwidth=0,
            highlightthickness=0,
            bg=bg_color,
            activebackground=bg_color,
            command=lambda: self.stop_button_pressed()
        )
        self.button_stop.place(
            x=64.0,
            y=778.0,
            width=512.0,
            height=110.0
        )

        # START button
        self.button_start_img = tk.PhotoImage(
            file=self.relative_to_assets("button_start.png"))
        button_start = tk.Button(
            image=self.button_start_img,
            borderwidth=0,
            highlightthickness=0,
            bg=bg_color,
            activebackground=bg_color,
            command=lambda: self.start_button_pressed()
        )
        button_start.place(
            x=64.0,
            y=690.0,
            width=512.0,
            height=72.0
        )

        # Input X coordinate
        self.entry_x_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_x.png"))
        self.canvas.create_image(
            320.0,
            266,
            image=self.entry_x_img
        )
        self.entry_x = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0,
        )
        self.entry_x.place(
            x=210,
            y=245,
            width=256,
            height=46
        )

        # Input Y coordinate
        self.entry_y_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_y.png"))
        self.canvas.create_image(
            320.0,
            343.5,
            image=self.entry_y_img
        )
        self.entry_y = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0,
        )
        self.entry_y.place(
            x=210,
            y=323,
            width=256,
            height=46
        )

        # Input Z coordinate
        self.entry_z_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_z.png"))
        self.canvas.create_image(
            320.0,
            420.5,
            image=self.entry_z_img
        )
        self.entry_z = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0
        )
        self.entry_z.place(
            x=210,
            y=400,
            width=256,
            height=46
        )

        # Input time
        self.entry_time_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_time.png"))
        self.canvas.create_image(
            320.0,
            522.0,
            image=self.entry_time_img
        )
        self.entry_time = tk.Entry(
            bd=0,
            bg=bg_color,
            fg=fg_color,
            font=entry_font,
            highlightthickness=0
        )
        self.entry_time.place(
            x=210,
            y=501,
            width=256,
            height=46
        )

        # Input type
        self.entry_path_type_img = tk.PhotoImage(
            file=self.relative_to_assets("entry_type.png"))
        self.canvas.create_image(
            320.0,
            600,
            image=self.entry_path_type_img
        )

        # Combo box for task type selection
        combostyle = ttk.Style()
        combostyle.theme_create('combostyle', parent='alt',
                                settings = {'TCombobox':
                                            {'configure':
                                            {"anchor": "se",
                                            "relief": "flat",
                                            'selectbackground': bg_color,
                                            'fieldbackground': bg_color,
                                            "foreground": fg_color,
                                            "arrowcolor": "false",
                                            }}}
                                )
        combostyle.theme_use('combostyle')

        options = ["", 
                   conf.P2P_DIRECT_TRAJECTORY,
                   conf.P2P_JOINT_TRAJECTORY,
                   conf.P2P_CONTINUOUS_TRAJECTORY,
                   conf.PICK_TRAJECTORY,
                   conf.PLACE_TRAJECTORY,
                   conf.HOMING]
        
        self.combo_task_type = ttk.Combobox(self.window, 
            values=options,
            background=bg_color,
            foreground=fg_color,
            state="readonly"
        )

        self.combo_task_type.configure(
            background=bg_color,
            foreground=fg_color,
            font=entry_font
        )
        self.combo_task_type.place(
            x= 190,
            y= 577,
            width= 350,
            height= 45
        )

        # init popup window
        self.popup = None

        # spin the gui
        self.window.mainloop()
        return

    # ***********************************************************************************


    def start_button_pressed(self):
        """
        Handles the start button press event.
        """

        # If there is no lock, it can move
        if self.pub_task_lock == False:
            # if task is homing
            if str(self.combo_task_type.get()) == conf.HOMING:
                self.task_homing()
                return

            trajectory_task_msg = TrajectoryTask()

            # Get trajectory task from GUI
            try:
                trajectory_task_msg.pos_end.x       = float(self.entry_x.get())
                trajectory_task_msg.pos_end.y       = float(self.entry_y.get())
                trajectory_task_msg.pos_end.z       = float(self.entry_z.get())
                trajectory_task_msg.task_time       = float(self.entry_time.get())
                trajectory_task_msg.task_type.data  = str(self.combo_task_type.get())
                trajectory_task_msg.is_trajectory_absolute_coordinates = True
            except:
                self.get_logger().error("Insert valid input")

            # Publish task
            self.trajectory_task_input_pub.publish(trajectory_task_msg)
            # set a lock to publish once
            self.pub_task_lock = True

        else:
            # Manage exception
            self.get_logger().warning("pub_task_lock is True!")

        return


    def stop_button_pressed(self):
        """
        Handles the stop button press event.
        """
        self.publish_robot_state(conf.ROBOT_STATE_STOP)
        # set a lock for publishing new tasks
        self.pub_task_lock = True
        
        self.raise_exception(conf.ROBOT_STATE_STOP)
        return


    def robot_state_callback(self, robot_state_msg):
        """
        Callback function for receiving robot state.
        """
        if robot_state_msg.data == conf.ROBOT_STATE_IDLE:
            # remove lock for publishing new tasks
            self.pub_task_lock = False
        
        elif robot_state_msg.data == conf.ROBOT_STATE_ERROR:
            self.raise_exception(conf.ROBOT_STATE_ERROR)
            # set a lock for publishing new tasks
            self.pub_task_lock = True

        else:
            # set a lock for publishing new tasks
            self.pub_task_lock = True

        return


    def publish_robot_state(self, state):
        """
        Publishes the robot state.
        """
        # Publish robot state
        robot_state_output_msg = String()
        robot_state_output_msg.data = state
        self.robot_state_pub.publish(robot_state_output_msg)
        return



    def raise_exception(self, exception):
        """
        Raises an exception.
        
        Args:
            exception (str): The description of the exception.
        
        Returns:
            None
        """
        if self.popup and self.popup.winfo_exists():
            self.popup.destroy()  # Close the existing popup if it's still open


        self.popup = tk.Toplevel()
        self.popup.title("Exception")
        
        # Center the popup window
        self.popup.geometry("400x200+{}+{}".format(
            int(self.popup.winfo_screenwidth() / 2 - 200),
            int(self.popup.winfo_screenheight() / 2 - 100))
        )

        # Set background color to blue
        self.popup.configure(bg="#2B3499")

        label = ttk.Label(self.popup, 
                         text=f"The robot state is\n{str(exception).upper()}",
                         font=("Helvetica", 16),
                         foreground="white",  # Set text color to white
                         background="#2B3499")  # Set label background color to blue
        label.pack(pady=40)

        button = ttk.Button(self.popup, text="CONTINUE", command=self.solve_exception,
                            style="Orange.TButton")  # Apply custom style
        button.place(
            x=40,
            y=120,
            width=320,
            height=40
        )

        # Create custom style for the button
        style = ttk.Style()
        style.configure("Orange.TButton", foreground="orange", background="#2B3499", font=("Helvetica", 18),
                        padding=(80, 5))

        return
    

    def solve_exception(self):
        """
        Resolves an exception.
        """
        self.publish_robot_state(conf.ROBOT_STATE_IDLE)
        
        # removes the lock for publishing new tasks
        self.pub_task_lock = False
        
        if self.popup and self.popup.winfo_exists():
            self.popup.destroy()
        return
    

    def update_display_position_callback(self, task_msg):
        # display x
        x = round(task_msg.pos_end.x, 3)
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, str(x))

        # display y
        y = round(task_msg.pos_end.y, 3)
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, str(y))

        # display z
        z = round(task_msg.pos_end.z, 3)
        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, str(z))

        return
    
    def task_homing(self):
        pos_current = conf.configuration["trajectory"]["pos_home"]     ## after home calibration

        homing_msg = Bool()
        homing_msg.data = True
        self.homing_pub.publish(homing_msg)

        # set a lock for publishing new tasks
        self.pub_task_lock = True

        ## update gui
        # display x
        x = round(pos_current[0], 3)
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, str(x))

        # display y
        y = round(pos_current[1], 3)
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, str(y))

        # display z
        z = round(pos_current[2], 3)
        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, str(z))
        return


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUI()

    # Create a thread for running the Tkinter main loop
    gui_thread = threading.Thread(target=gui_node.init_gui)
    gui_thread.daemon = True  # Make the thread a daemon so it terminates when the main thread terminates
    gui_thread.start()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()