#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import String

import cv2 as cv
import numpy as np

import deltarobot_inputs.cv_conf as cv_conf

import time


class ObjectDetection(Node):

    def __init__(self):

        super().__init__('object_detection_node')

        #**********************************************************#
        #                     define publishers                    #
        #**********************************************************#
        self.input_gcode_cmds__pub = self.create_publisher(
            String,
            'input_gcode_cmds',
            10)

        #**********************************************************#
        #                     define subscribers                   #
        #**********************************************************#
        self.task_scheduler_queue_len__sub = self.create_subscription(
            Int16,
            'task_scheduler_queue_len',
            self.task_scheduler_queue_len__callback,
            1)

        #*************  !! set to -1 !! *************
        self.task_scheduler_queue_len = -1


        #**********************************************************#
        #                     define timers                        #
        #**********************************************************#
        timer_period = 0.01 # seconds --> 1/60fps
        self.read_frame_timer = self.create_timer(
            timer_period,
            self.read_frame__timer_callback)
        self.current_frame = None


        timer_period = 0.1 # seconds
        self.detect_objects_timer = self.create_timer(
            timer_period,
            self.detect_objects__timer_callback)


        #**********************************************************#
        #                 define class attributes                  #
        #**********************************************************#
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

        ## go to home before starting (for safety)
        time.sleep(0.01)
        init_msg = String()
        init_msg.data = "G28"
        self.input_gcode_cmds__pub.publish(init_msg)

        time.sleep(0.01)
        init_msg = String()
        init_msg.data = "G01 X0 Y50 Z-120 T5"
        self.input_gcode_cmds__pub.publish(init_msg)
        return

    ###################################################################################
    #                                                                                 #
    #                              CALLBACK FUNCTIONS                                 #
    #                                                                                 #
    ###################################################################################
    def read_frame__timer_callback(self):
        ret, frame = self.video_capture.read()

        if not ret:
            self.get_logger().error("Could not read frame")
            self.current_frame = None

        ## *****************  show frames  ***************************
        if cv_conf.SHOW_FRAME:
            cv.waitKey(1)
            cv.imshow("frame", frame)
        ## *********************************************************************

        self.current_frame = frame
        return


    def detect_objects__timer_callback(self):
        if self.current_frame is None:
            return
        
        ## robot is ready for new tasks
        if self.task_scheduler_queue_len == 0:

            washer_position_list = self.get_objects_position_list(self.current_frame)

            if washer_position_list is None:
                self.get_logger().info("No objects detected")
                return

            ## create motion planning
            program = self.get_motion_planning_program(washer_position_list)

            print(program)

            ## publish gcode cmds
            self.input_gcode_cmds__publish(program)

        return


    def task_scheduler_queue_len__callback(self, msg):
        self.task_scheduler_queue_len = msg.data
        return



    ###################################################################################
    #                                                                                 #
    #                             PUBLISHING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def input_gcode_cmds__publish(self, program):

        for task in program:
            time.sleep(0.001)
            msg = String()
            msg.data = task

            self.input_gcode_cmds__pub.publish(msg)
            print(task)

        return

    ###################################################################################
    #                                                                                 #
    #                               UTILS FUNCTIONS                                   #
    #                                                                                 #
    ###################################################################################

    def init_IP_camera(self, CAMERA_IP_ADDRESS):
        ## try for n times until connection was succesful
        total_attempts = 5
        for i in range(total_attempts):
            self.video_capture = cv.VideoCapture(CAMERA_IP_ADDRESS)

            # Check if the connection was successful
            if not self.video_capture.isOpened():
                self.get_logger().warning(f"Could not open video stream [attempt {i}]")
                time.sleep(2)   # wait to try later

            else:
                self.get_logger().info("Video stream opened successfully")
                return True    # no connection error

        ## couldn't enable camera connection
        self.get_logger().error(f"Could not open video stream")
        return False     # is connection error


    ###################################################################################
    #                                                                                 #
    #                        COMPUTER VISION FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def get_objects_position_list(self, img):

        ## detect markers
        corners, ids = self.detect_markers(img)
        if corners is None:
            return None

        ## transform image
        img_dst = self.transform_image(img, corners, ids)
        if img_dst is None:
            return None

        ## detect circles
        washer_position_list = self.detect_circles(img_dst)
        if washer_position_list is None:
            return None

        return washer_position_list
    

    def detect_markers(self, img):
        ## apply filters to image
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        
        corners, ids, RejectedImgPoints = cv.aruco.detectMarkers(gray_img, self.aruco_dict)
        
        if ids is None or corners is None:
            return None, None

        if len(ids) != 4:   ## not all four markers have been detected
            self.get_logger().warning("Markers not detected")
            return None, None
        

        ## *****************  mark detected markers  ***************************
        if cv_conf.SHOW_MARKERS:
            img_marker = cv.aruco.drawDetectedMarkers(img, corners, ids)

            cv.waitKey(1)
            cv.imshow("detected markers", img_marker)
        ## *********************************************************************

        return corners, ids


    def transform_image(self, img, corners, ids):
        pts1 = np.empty((4,2), dtype=np.float32)
        pts2 = np.float32([[0,0],
                           [cv_conf.WS_LENGTH,  0],
                           [0,                  cv_conf.WS_HEIGHT],
                           [cv_conf.WS_LENGTH,  cv_conf.WS_HEIGHT]])

        # extract x,y coordinates of detected markers
        for id in ids:
            id = id[0]

## ********  !! check if this chunk can be removed !! ********
            if id == 0:
                corners_id = 2
            elif id == 1:
                corners_id = 3
            elif id == 2:
                corners_id = 0
            elif id == 3:
                corners_id = 1
            else:
                return None
## ***********************************************************

            x = corners[corners_id][0][0][0]
            y = corners[corners_id][0][0][1]

            pts1[id,:] = np.float32([x,y])

        M = cv.getPerspectiveTransform(pts1, pts2)
        dst = cv.warpPerspective(img, M, (cv_conf.WS_LENGTH, cv_conf.WS_HEIGHT))

        return dst


    def detect_circles(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        edges = cv.Canny(blurred, 50, 150)

        # Find contours in the edge-detected image
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        ## from experiment to dect a washer
        total_area = len(img[0])*len(img)
        min_area = 0.005*total_area
        max_area = 0.05*total_area

        washer_position_list = []

        ## find circles based on perimeter and area
        for contour in contours:
            area = cv.contourArea(contour)

            # if area is not in threshold
            if area < min_area or area > max_area:
                continue

            perimeter = cv.arcLength(contour, True)
            diameter = perimeter / np.pi
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity > 0.85:  # adjust circularity threshold as needed
                (x, y, w, h) = cv.boundingRect(contour)
                x = x + w/2
                y = y + h/2
                washer_position_list.append([int(x), int(y), diameter, circularity])

        ## no washers have been detected
        if not washer_position_list:
            return None
        

        ## *****************  show edges  ***************************
        if cv_conf.SHOW_EDGES:    
            cv.waitKey(1)
            cv.imshow("edges", edges)
        ## *********************************************************************

        ## *****************  mark detected objects  ***************************
        if cv_conf.SHOW_OBJECTS:
            for washer_pos in washer_position_list:
                x = washer_pos[0]
                y = washer_pos[1]
                diameter = washer_pos[2]
                circularity = washer_pos[3]
                
                self.get_logger().info("***** objects detected at: *****")
                self.get_logger().info(f"x: {x}, y: {y}")

                img = cv.rectangle(img, 
                                   pt1=(int(x-diameter/2), int(y-diameter/2)), 
                                   pt2=(int(x+diameter/2), int(y+diameter/2)), 
                                   color=(0, 255, 0), 
                                   thickness=2)
                img = cv.putText(img,
                                 text=f"certainty: {round(circularity*100 ,1)}%", 
                                 org=(int(x+diameter/2), int(y-diameter/2+20)),
                                 fontFace=1,
                                 fontScale=1,
                                 color=(0, 0, 255),
                                 thickness=1,
                                 lineType=2)
                img = cv.putText(img,
                                 text=f"diameter: {round(diameter / cv_conf.img_dst_prescaler ,0)}", 
                                 org=(int(x+diameter/2), int(y-diameter/2)),
                                 fontFace=1,
                                 fontScale=1,
                                 color=(0, 0, 255),
                                 thickness=1,
                                 lineType=2)
                # img_dst = cv.circle(img_dst, (x, y), 2, (0, 0, 255), 2)

            cv.waitKey(1)
            cv.imshow("detected objects", img)
        ## *********************************************************************


        return washer_position_list



    ###################################################################################
    #                                                                                 #
    #                        MOTION PLANNING FUNCTIONS                                #
    #                                                                                 #
    ###################################################################################

    def get_motion_planning_program(self, object_pos_list):
        motion_planning_program = []
        cmd = ""

        for object_pos in object_pos_list:

            self.get_logger().info(f"************** {object_pos} **************")

            ## *********  pick object **********
            # go over object
            x = cv_conf.marker0_x - (object_pos[0]/cv_conf.img_dst_prescaler)
            y = cv_conf.marker0_y - (object_pos[1]/cv_conf.img_dst_prescaler)
            z = cv_conf.z_work_plane + cv_conf.z_offset
            cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
            motion_planning_program.append(cmd)

            # go to object
            z = cv_conf.z_work_plane + cv_conf.washer_thickness
            cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
            motion_planning_program.append(cmd)

            # close gripper
            cmd = f"M03"
            motion_planning_program.append(cmd)

            # return over object
            z = cv_conf.z_work_plane + cv_conf.z_offset
            cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
            motion_planning_program.append(cmd)


            ## *********  place object **********
            # go to box0
            x = cv_conf.box0_x
            y = cv_conf.box0_y
            z = cv_conf.box0_z
            cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
            motion_planning_program.append(cmd)

            # open gripper
            cmd = f"M05"
            motion_planning_program.append(cmd)

            # go over box0
            x = cv_conf.box0_x
            y = cv_conf.box0_y
            z = cv_conf.box0_z + cv_conf.z_offset
            cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
            motion_planning_program.append(cmd)


        # ## return to idle position
        #     x = cv_conf.idle_pos_x
        #     y = cv_conf.idle_pos_y
        #     z = cv_conf.idle_pos_z
        #     cmd = f"G01 X{x} Y{y} Z{z} T{-1}"
        #     motion_planning_program.append(cmd)

        return motion_planning_program





###################################################################################
#                                                                                 #
#                                    MAIN                                         #
#                                                                                 #
###################################################################################


def main(args=None):
    rclpy.init(args=args)

    object_detection_node = ObjectDetection()
    
    CAMERA_IP_ADDRESS = "http://192.168.1.186:4747/video"
    # CAMERA_IP_ADDRESS = "http://10.42.0.13:4747/video"
    connection_ack = object_detection_node.init_IP_camera(CAMERA_IP_ADDRESS)

    if not connection_ack:
        object_detection_node.destroy_node()
        rclpy.shutdown()
        return


    rclpy.spin(object_detection_node)

    object_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()