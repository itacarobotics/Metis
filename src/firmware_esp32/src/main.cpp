#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "board_pinout.h"
#include "motor_driver/motor_driver.h"
#include "error_log/error_log.h"
#include "uros_subscribers/callback_functions.h"
#include "uros_publishers/uros_publishers.h"

#include "robot_interfaces.h"

void setup()
{
// configure wifi transport
    // IPAddress agent_ip(192, 168, 113, 175);
    // size_t agent_port = 8888;
    // char ssid[] = "Laptop di Fede";
    // char psk[] = "12345678";
    // set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    // configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(500);

    // initialize shift register pinout
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_CLOCK_PIN, OUTPUT);
    pinMode(I2S_LATCH_PIN, OUTPUT);

    // initialize limit switches pinout
    pinMode(LIMIT_SWITCH_1_PIN, INPUT);
    pinMode(LIMIT_SWITCH_2_PIN, INPUT);
    pinMode(LIMIT_SWITCH_3_PIN, INPUT);

    // initialize gripper pinout
    // ...

    // micro-ROS setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(  &support, 
                                0, 
                                NULL, 
                                &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default( &node, 
                                    "micro_controller_node",
                                    "", 
                                    &support));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init( &executor, 
                                &support.context, 
                                3,                      // number of subscribers, timers, ...
                                &allocator));


    /********************************************************************************************************************
     *
     *                            robot_cmds/move/joint_trajectory subscriber
     *
     ********************************************************************************************************************/
    rcl_subscription_t robot_cmds__move__joint_trajectory__sub;
    RCCHECK(rclc_subscription_init_default( &robot_cmds__move__joint_trajectory__sub, &node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(micro_custom_messages, msg, JointTrajectoryArray),
                                            "robot_cmds/move/joint_trajectory"));

    // allocate message memory
    micro_custom_messages__msg__JointTrajectoryArray via_points_array_msg;  // allocate message memory
    via_points_array_msg.array_size = 400;                                  // max capacity
    via_points_array_msg.via_points.capacity = via_points_array_msg.array_size;
    via_points_array_msg.via_points.data = (micro_custom_messages__msg__JointTrajectory*)malloc(
    via_points_array_msg.via_points.capacity * sizeof(micro_custom_messages__msg__JointTrajectory));
    via_points_array_msg.via_points.size = 0;

    // add subscription to topic
    RCCHECK(rclc_executor_add_subscription( &executor, 
                                            &robot_cmds__move__joint_trajectory__sub, 
                                            &via_points_array_msg,
                                            &robot_cmds__move__joint_trajectory__callback, 
                                            ON_NEW_DATA));


    /********************************************************************************************************************
    *
    *                                    robot_cmds/homing SUBSCRIBER
    *
    ********************************************************************************************************************/
    rcl_subscription_t robot_cmds__homing__sub;
    RCCHECK(rclc_subscription_init_default( &robot_cmds__homing__sub, &node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), 
                                            "robot_cmds/homing"));

    // allocate message memory
    std_msgs__msg__Bool robot_cmds__homing__msg;

    // add subscription to topic
    RCCHECK(rclc_executor_add_subscription( &executor, 
                                            &robot_cmds__homing__sub, 
                                            &robot_cmds__homing__msg,
                                            &robot_cmds__homing__callback, ON_NEW_DATA));


    /********************************************************************************************************************
    *
    *                                    robot_cmds/gripper/em SUBSCRIBER
    *
    ********************************************************************************************************************/
    rcl_subscription_t robot_cmds__gripper__em__sub;
    RCCHECK(rclc_subscription_init_default( &robot_cmds__gripper__em__sub, 
                                            &node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), 
                                            "robot_cmds/gripper/em"));

    // allocate message memory
    std_msgs__msg__Bool robot_cmds__gripper__em__msg;

    // add subscription to topic
    RCCHECK(rclc_executor_add_subscription( &executor, 
                                            &robot_cmds__gripper__em__sub, 
                                            &robot_cmds__gripper__em__msg,
                                            &robot_cmds__gripper__em__callback, 
                                            ON_NEW_DATA));


    /********************************************************************************************************************
    *
    *                                    feedback/task_ack PUBLISHER
    *
    ********************************************************************************************************************/
    RCCHECK(rclc_publisher_init_default(&feedback__task_ack__pub, 
                                        &node, 
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                        "feedback/task_ack"));


    // spin the node
    rclc_executor_spin(&executor);  // loop*

    // destroy nodes
    RCCHECK(rcl_subscription_fini(&robot_cmds__move__joint_trajectory__sub, &node));
    RCCHECK(rcl_subscription_fini(&robot_cmds__homing__sub, &node));
    RCCHECK(rcl_subscription_fini(&robot_cmds__gripper__em__sub, &node));
    RCCHECK(rcl_node_fini(&node));
    return;
}

void loop()
{
    // loop*
}