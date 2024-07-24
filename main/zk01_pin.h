#ifndef ZK01_PIN_H
#define ZK01_PIN_H

#include "driver/gpio.h"
// external signal indicate rolling direction
#define UP_PIN GPIO_NUM_19
#define DOWN_PIN GPIO_NUM_20

// RS485 remote control to this board
#define RS485_DIR_PIN GPIO_NUM_18

// mcu used to drive h-bridge
#define UP_EN_PIN GPIO_NUM_2
#define DOWN_EN_PIN GPIO_NUM_3
#define UP_PWM_PIN GPIO_NUM_7
#define DOWN_PWM_PIN GPIO_NUM_22

// motor feed back signal
#define STALL_PIN GPIO_NUM_23
#define P_E0_PIN GPIO_NUM_14
#define P_E1_PIN GPIO_NUM_15

// system set button
#define SET_PIN GPIO_NUM_13

// status indicator
#define LED_RUN1_PIN GPIO_NUM_12

#endif // ZK01_PIN_H