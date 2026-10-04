/*
 * DC_motor_control.h
 *
 *  Created on: 21 sie 2026
 *      Author: ksapi
 */

#ifndef COMPONENTS_DC_MOTOR_CONTROL_H_
#define COMPONENTS_DC_MOTOR_CONTROL_H_

#include <stdint.h>
#include "driver/mcpwm_types.h"
#include "driver/pulse_cnt.h"
#include "soc/gpio_num.h"
#include "sys/types.h"


#define MCPWM_TIMER_RESOLUTION_HZ 8000000 // 8MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ             20000  // 20KHz PWM
#define MCPWM_DUTY_TICK_MAX       (MCPWM_TIMER_RESOLUTION_HZ / MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks (400)
#define HOLDING_PWM_TRESHOLD      0

#define ENCODER_GPIO_A            36
#define ENCODER_GPIO_B            35
#define ENCODER_PCNT_HIGH_LIMIT   32767
#define ENCODER_PCNT_LOW_LIMIT    -32768
#define ENCODER_MAX_GLITCH		  1000  // in ns



typedef struct{
	pcnt_unit_handle_t pcnt_encoder;
	mcpwm_cmpr_handle_t cmpr_ptr;
	int16_t	 pwm_cmpr_value;
	uint8_t motor_in_gpio_right;
	uint8_t motor_in_gpio_left;
} motor_control_context_t;

void motor_mcpwm_init(mcpwm_cmpr_handle_t *cmpr_A_ptr, mcpwm_cmpr_handle_t *cmpr_B_ptr, gpio_num_t GPIO_wave_A, gpio_num_t GPIO_wave_B);

void vMotorControlTask(void *pvParameters);

#endif /* COMPONENTS_DC_MOTOR_CONTROL_H_ */