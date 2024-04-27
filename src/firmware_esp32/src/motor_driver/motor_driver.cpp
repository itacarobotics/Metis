#include "motor_driver.h"

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);


MotorDriver::MotorDriver(void)
{
    // initialize stepper pins to LOW
    sr.set(PIN_STEPPERS_STEP[STEPPER1_IDX], PIN_STEPPERS_STEP__state[STEPPER1_IDX]);
    sr.set(PIN_STEPPERS_STEP[STEPPER2_IDX], PIN_STEPPERS_STEP__state[STEPPER2_IDX]);
    sr.set(PIN_STEPPERS_STEP[STEPPER3_IDX], PIN_STEPPERS_STEP__state[STEPPER3_IDX]);

    return;
}


void MotorDriver::toggle_stepper_step_tick(int stepper_index)
{
    if (PIN_STEPPERS_STEP__state[stepper_index] == LOW){
        sr.set(PIN_STEPPERS_STEP[stepper_index], HIGH);
        PIN_STEPPERS_STEP__state[stepper_index] = HIGH;
    }
    else {
        sr.set(PIN_STEPPERS_STEP[stepper_index], LOW);
        PIN_STEPPERS_STEP__state[stepper_index] = LOW;
    }

    return;
}


void MotorDriver::set_direction(char PIN_STEPPER_X_DIR, float sign)
{
    if (sign < 0) {
        sr.set(PIN_STEPPER_X_DIR, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_DIR, LOW);
    }
    return;
}


int MotorDriver::get_total_stepper_ticks(float delta_q_mm, int stepper_index)
{
    int stepper_ticks_total;

    // adding error compensation
    // delta_q_mm += delta_q_remainder_mm[stepper_index];
    // calculating ticks, where two ticks are one (micro-)step
    stepper_ticks_total = 2.0F * delta_q_mm / MM_PER_STEP;
    
    // updating error compensation
    // delta_q_remainder_mm[stepper_index] = delta_q_mm - (0.5F * float(stepper_ticks_total) * MM_PER_STEP);

    return abs(stepper_ticks_total);
}


int MotorDriver::get_delay_stepper_ticks(int stepper_ticks_total, int delta_t_micros)
{
    int stepper_tick_delay_micros;

    if (stepper_ticks_total != 0) {  // avoid zero division
        stepper_tick_delay_micros = delta_t_micros / stepper_ticks_total;
    }
    else {
        stepper_tick_delay_micros = 0;
    }

    return stepper_tick_delay_micros;
}

/*
    moving three stepper motors at different speeds in same interval of time.
*/
void MotorDriver::move_steppers_async(float delta_q1_mm, float delta_q2_mm, float delta_q3_mm, int delta_t_micros)
{
    // stepper 1
    set_direction(PIN_STEPPER_1_DIR, delta_q1_mm);
    int stepper1_ticks_counter      = 1;
    int stepper1_ticks_total        = get_total_stepper_ticks(delta_q1_mm, STEPPER1_IDX);
    int stepper1_tick_delay_micros  = get_delay_stepper_ticks(stepper1_ticks_total, delta_t_micros);
    
    // stepper 2
    set_direction(PIN_STEPPER_2_DIR, delta_q2_mm);
    int stepper2_ticks_counter      = 1;
    int stepper2_ticks_total        = get_total_stepper_ticks(delta_q2_mm, STEPPER2_IDX);
    int stepper2_tick_delay_micros  = get_delay_stepper_ticks(stepper2_ticks_total, delta_t_micros);
    
    // stepper 3
    set_direction(PIN_STEPPER_3_DIR, delta_q3_mm);
    int stepper3_ticks_counter      = 1;
    int stepper3_ticks_total        = get_total_stepper_ticks(delta_q3_mm, STEPPER3_IDX);
    int stepper3_tick_delay_micros  = get_delay_stepper_ticks(stepper3_ticks_total, delta_t_micros);


    /**********************************************************
     *                      CONTROL STEPPERS
     **********************************************************/

    int time_dt = 0;
    int initial_time = micros();

    while (true)
    {
        time_dt = micros() - initial_time;

        // stepper 1
        if ((stepper1_ticks_counter <= stepper1_ticks_total) &&
            (time_dt >= stepper1_tick_delay_micros * stepper1_ticks_counter)) {

            toggle_stepper_step_tick(STEPPER1_IDX);
            stepper1_ticks_counter += 1;

        }

        // stepper 2
        else if ((stepper2_ticks_counter <= stepper2_ticks_total) &&
                 (time_dt >= stepper2_tick_delay_micros * stepper2_ticks_counter)) {

            toggle_stepper_step_tick(STEPPER2_IDX);
            stepper2_ticks_counter += 1;

        }

        // stepper 3
        else if ((stepper3_ticks_counter <= stepper3_ticks_total) &&
                 (time_dt >= stepper3_tick_delay_micros * stepper3_ticks_counter)) {

            toggle_stepper_step_tick(STEPPER3_IDX);
            stepper3_ticks_counter += 1;

        }

        if( (stepper1_ticks_counter > stepper1_ticks_total) && 
            (stepper2_ticks_counter > stepper2_ticks_total) &&
            (stepper3_ticks_counter > stepper3_ticks_total))
        {
            break;
        }
    }

    return;
}


void MotorDriver::homing(void)
{
    bool is_homing_1 = true;
    bool is_homing_2 = true;
    bool is_homing_3 = true;

    set_direction(PIN_STEPPER_1_DIR, 1.0F);
    set_direction(PIN_STEPPER_2_DIR, 1.0F);
    set_direction(PIN_STEPPER_3_DIR, 1.0F);

    while (true) {
        // if all limit switches are pressed
        if (!is_homing_1 && !is_homing_2 && !is_homing_3) {
            break;
        }

        // check if limit switch is pressed
        if (analogRead(LIMIT_SWITCH_1_PIN) > 0) {
            is_homing_1 = false;
        }
        if (analogRead(LIMIT_SWITCH_2_PIN) > 0) {
            is_homing_2 = false;
        }
        if (analogRead(LIMIT_SWITCH_3_PIN) > 0) {
            is_homing_3 = false;
        }

        // do half step
        if (is_homing_1) {
            toggle_stepper_step_tick(STEPPER1_IDX);
        }
        if (is_homing_2) {
            toggle_stepper_step_tick(STEPPER2_IDX);
        }
        if (is_homing_3) {
            toggle_stepper_step_tick(STEPPER3_IDX);
        }

        // wait
        delayMicroseconds(700);
    }

    delay(500);

    // go back by default offset
    set_direction(PIN_STEPPER_1_DIR, -1.0F);
    set_direction(PIN_STEPPER_2_DIR, -1.0F);
    set_direction(PIN_STEPPER_3_DIR, -1.0F);

    for (int i = 0; i < 300; ++i) {
        toggle_stepper_step_tick(STEPPER1_IDX);
        toggle_stepper_step_tick(STEPPER2_IDX);
        toggle_stepper_step_tick(STEPPER3_IDX);
        delayMicroseconds(700);
    }

    return;
}