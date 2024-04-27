#include "callback_functions.h"

MotorDriver mdriver;

void robot_cmds__move__joint_trajectory__callback(const void* msgin)
{
    // unpack message trajectory
    micro_custom_messages__msg__JointTrajectoryArray* via_points_array_msg =
        (micro_custom_messages__msg__JointTrajectoryArray*)msgin;

    size_t n_via_points = via_points_array_msg->array_size;

    float delta_q1;
    float delta_q2;
    float delta_q3;
    float delta_time;
    unsigned long int delta_time_micros;

    for (int via_point_index = 0; via_point_index < (n_via_points - 1); ++via_point_index)
    {
        // stepper 1
        delta_q1 = via_points_array_msg->via_points.data[via_point_index + 1].q1 -
        via_points_array_msg->via_points.data[via_point_index].q1;

        // stepper 2
        delta_q2 = via_points_array_msg->via_points.data[via_point_index + 1].q2 -
        via_points_array_msg->via_points.data[via_point_index].q2;

        // stepper 3
        delta_q3 = via_points_array_msg->via_points.data[via_point_index + 1].q3 -
        via_points_array_msg->via_points.data[via_point_index].q3;

        // delta time
        delta_time = via_points_array_msg->via_points.data[via_point_index + 1].t_via_point -
        via_points_array_msg->via_points.data[via_point_index].t_via_point;
        delta_time_micros = delta_time * 1000000;

        mdriver.move_steppers_async(delta_q1, 
                                    delta_q2, 
                                    delta_q3,
                                    delta_time_micros);
    }

    // publish ack
    std_msgs__msg__Bool task_ack_msg;
    task_ack_msg.data = true;
    RCCHECK(rcl_publish(&feedback__task_ack__pub, &task_ack_msg, NULL));

    return;
}

void robot_cmds__homing__callback(const void* msgin)
{
    std_msgs__msg__Bool* task_homing_msg = (std_msgs__msg__Bool*)msgin;

    if (task_homing_msg->data) {
        mdriver.homing();
    }
    else {
        // publish nak
        std_msgs__msg__Bool task_ack_msg;
        task_ack_msg.data = false;
        RCCHECK(rcl_publish(&feedback__task_ack__pub, &task_ack_msg, NULL));
    }

    // publish ack
    std_msgs__msg__Bool task_ack_msg;
    task_ack_msg.data = true;
    RCCHECK(rcl_publish(&feedback__task_ack__pub, &task_ack_msg, NULL));
}

void robot_cmds__gripper__em__callback(const void* msgin)
{
    std_msgs__msg__Bool* msg = (std_msgs__msg__Bool*)msgin;

    if (msg->data) {
        analogWrite(ELECTRO_MAGNET_PIN, 0);
    }
    else {
        analogWrite(ELECTRO_MAGNET_PIN, 70);
    }

    // publish ack
    std_msgs__msg__Bool task_ack_msg;
    task_ack_msg.data = true;
    RCCHECK(rcl_publish(&feedback__task_ack__pub, &task_ack_msg, NULL));
}