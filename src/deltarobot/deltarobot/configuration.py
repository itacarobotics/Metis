#!/usr/bin/env python3

import numpy as np


## absolute paths
package_path = "/home/ostifede02/dr_ws/src/deltarobot_description"
gui_assets_path = "/home/ostifede02/dr_ws/src/deltarobot_inputs/assets/gui/"

## urdf filenames
urdf_filename_1 = "deltarobot_1.urdf"   # chain 1
urdf_filename_2 = "deltarobot_2.urdf"   # chain 2
urdf_filename_3 = "deltarobot_3.urdf"   # chain 3

## inverse geometry frame IDs
frame_id_1 = 10   # chain 1
frame_id_2 = 10   # chain 2
frame_id_3 = 10   # chain 3


## pos home
pos_home = np.array([0, 0, -45])

## joint limits
q_min = 0     # [ mm ]
q_max = 500     # [ mm ]

## trajectory limits
max_vel                 = 1500   # [ mm/s ]
max_acc                 = 250   # [ mm/s2 ]
via_points_distance     = 2.6     # [ mm ]
via_points_threshold    = 10    # [ mm ]

## physical properties
# link_length = 400     --> set in deltarobot_description
end_effector_radius = 42    # [ mm ]



##########   TRAJECTORY ROUTINES   ##########
PTP_TASK_SPACE_TRAJECTORY       = "PTP_TASK_SPACE_TRAJECTORY"
PTP_JOINT_SPACE_TRAJECTORY      = "PTP_JOINT_SPACE_TRAJECTORY"
HOMING                          = "HOMING"
GRIPPER_OPEN                    = "GRIPPER_OPEN"
GRIPPER_CLOSED                  = "GRIPPER_CLOSED"


##########   ROBOT STATES   ##########
ROBOT_STATE_IDLE        = "idle"
ROBOT_STATE_RUN         = "run"
ROBOT_STATE_STOP        = "stop"
ROBOT_STATE_ERROR       = "error"
