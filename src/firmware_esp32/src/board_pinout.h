#ifndef BOARD_PINOUT_H
#define BOARD_PINOUT_H

// shift register pinout
#define NUMBER_SHIFT_REG 1
#define I2S_DATA_PIN 21
#define I2S_CLOCK_PIN 16
#define I2S_LATCH_PIN 17

// motor driver pinout
#define PIN_STEPPER_1_STEP 1
#define PIN_STEPPER_1_DIR 2

#define PIN_STEPPER_2_STEP 3
#define PIN_STEPPER_2_DIR 4

#define PIN_STEPPER_3_STEP 5
#define PIN_STEPPER_3_DIR 6

// limit switch pinout
#define LIMIT_SWITCH_1_PIN 36
#define LIMIT_SWITCH_2_PIN 35
#define LIMIT_SWITCH_3_PIN 34

// gripper pinout
#define ELECTRO_MAGNET_PIN 32

#endif  // BOARD_PINOUT_H