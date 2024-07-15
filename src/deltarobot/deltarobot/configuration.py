#!/usr/bin/env python3

import numpy as np

## workspace path
ws_path = "/home/ostifede02/itacarobotics/Metis"

## pos home
pos_home = np.array([0, 0, -45])

## joint limits
q_min = 0     # [ mm ]
q_max = 500     # [ mm ]

## trajectory limits
max_vel                 = 6000   # [ mm/s ]
max_acc                 = 1200   # [ mm/s2 ]
via_points_distance     = 1.5     # [ mm ]
via_points_threshold    = 2    # [ mm ]

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
