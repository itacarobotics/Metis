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

        super().__init__('gui_node')


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
        
        ## publish robot state
        self.robot_state__pub = self.create_publisher(
            String,
            'robot_state',
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
        

        # Initialize lock to avoid publishing double tasks
        self.pub_task_lock = False

        return
    


    ###################################################################################
    #                                                                                 #
    #                                 GUI APP LOOP                                    #
    #                                                                                 #
    ###################################################################################

    def gui_app_loop(self):
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
            command=lambda: self.stop__button_pressed()
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
            command=lambda: self.start__button_pressed()
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
                   conf.PTP_TASK_SPACE_TRAJECTORY,
                   conf.GRIPPER_OPEN,
                   conf.GRIPPER_CLOSED,
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


    ###################################################################################
    #                                                                                 #
    #                            BUTTON PRESSED FUNCTIONS                             #
    #                                                                                 #
    ###################################################################################

    def start__button_pressed(self):
        # If there is no lock, it can move
        if self.pub_task_lock == False:
            
            task_type  = str(self.combo_task_type.get())

            if task_type == conf.PTP_TASK_SPACE_TRAJECTORY:
                self.input_cmds__move__task_space__ptp__publish()
            elif task_type == conf.HOMING:
                self.input_cmds__homing__publish()
            elif task_type == conf.GRIPPER_OPEN:
                self.input_cmds__gripper__em__publish(True)
            elif task_type == conf.GRIPPER_CLOSED:
                self.input_cmds__gripper__em__publish(False)

            # set a lock to publish once
            self.pub_task_lock = True

        else:
            # Manage exception
            self.get_logger().warning("Publishing lock is True!")

        return

    def stop__button_pressed(self):

        self.robot_state__publish(conf.ROBOT_STATE_STOP)
        # set a lock for publishing new tasks
        self.pub_task_lock = True
        
        self.raise_exception__popup(conf.ROBOT_STATE_STOP)
        return


    ###################################################################################
    #                                                                                 #
    #                               EXCEPTIONS FUNCTIONS                              #
    #                                                                                 #
    ###################################################################################

    def raise_exception__popup(self, exception):

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

        button = ttk.Button(self.popup, text="CONTINUE", command=self.solve_exception__button_pressed,
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

    def solve_exception__button_pressed(self):
        # override robot state
        self.robot_state__publish(conf.ROBOT_STATE_IDLE)
        
        # removes the lock for publishing new tasks
        self.pub_task_lock = False

        # remove popup window        
        if self.popup and self.popup.winfo_exists():
            self.popup.destroy()

        return
    

    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################

    def robot_state__callback(self, msg):

        if msg.data == conf.ROBOT_STATE_IDLE:
            # remove lock for publishing new tasks
            self.pub_task_lock = False
        
        elif msg.data == conf.ROBOT_STATE_ERROR:
            self.raise_exception__popup(conf.ROBOT_STATE_ERROR)
            # set a lock for publishing new tasks
            self.pub_task_lock = True

        else:
            # set a lock for publishing new tasks
            self.pub_task_lock = True

        return


    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def robot_state__publish(self, state):
        # Publish robot state
        msg_out = String()
        msg_out.data = state
        self.robot_state__pub.publish(msg_out)
        return

    def input_cmds__move__task_space__ptp__publish(self):
        msg = TrajectoryTask()

        msg.pos_end.x       = float(self.entry_x.get())
        msg.pos_end.y       = float(self.entry_y.get())
        msg.pos_end.z       = float(self.entry_z.get())
        msg.time_total      = float(self.entry_time.get())

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


    ###################################################################################
    #                                                                                 #
    #                                    UTILS                                        #
    #                                                                                 #
    ###################################################################################
    
    def relative_to_assets(self, path):
        """
        Generates the absolute path to the assets folder.
        
        Parameters:
            path (str): The path to the asset.
        
        Returns:
            str: The absolute path to the asset.
        """
        ASSETS_PATH = conf.gui_assets_path
        return str(ASSETS_PATH + path)
    













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
        pos_current = conf.pos_home     ## after home calibration

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
    gui_thread = threading.Thread(target=gui_node.gui_app_loop)
    gui_thread.daemon = True  # Make the thread a daemon so it terminates when the main thread terminates
    gui_thread.start()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
