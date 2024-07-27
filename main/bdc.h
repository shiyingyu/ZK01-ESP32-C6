#ifndef BDC_H
#define BDC_H

#include "bdc_motor.h"
#include "bdc_motor_interface.h"
#include "driver/pulse_cnt.h"
#include "pid_ctrl.h"
#include "esp_timer.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000 // 25KHz
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A UP_PWM_PIN
#define BDC_MCPWM_GPIO_B DOWN_PWM_PIN

#define BDC_ENCODER_GPIO_A            P_E0_PIN
#define BDC_ENCODER_GPIO_B            P_E1_PIN
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS 10
#define BDC_PID_EXPECT_SPEED 400

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

extern motor_control_context_t motor_ctrl_ctx;
extern bdc_motor_handle_t motor;
extern esp_timer_handle_t pid_loop_timer;

void bdc_init();
void bdc_stop();
void bdc_up();
void bdc_down();

#endif // BDC_H