#ifndef CALLBACK_FUNCTIONS_H
#define CALLBACK_FUNCTIONS_H

// #include "stepper.h"
#include "motor_driver/motor_driver.h"
#include "robot_interfaces.h"
#include "board_pinout.h"
#include "error_log/error_log.h"
#include "uros_publishers/uros_publishers.h"

void robot_cmds__move__joint_trajectory__callback(const void* msgin);
void robot_cmds__gripper__em__callback(const void* msgin);
void robot_cmds__homing__callback(const void* msgin);

#endif  // CALLBACK_FUNCTIONS_H