#ifndef ERROR_LOG_H
#define ERROR_LOG_H

#include "Arduino.h"

#include "board_pinout.h"
#include "motor_driver/motor_driver.h"

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }

void error_loop(void);

#endif  // ERROR_LOG_H