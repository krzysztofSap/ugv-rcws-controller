/*
 * DC_motor_control.h
 *
 *  Created on: 21 sie 2026
 *      Author: ksapi
 */

#ifndef MAIN_DC_MOTOR_CONTROL_H_
#define MAIN_DC_MOTOR_CONTROL_H_


#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              15
#define BDC_HOLDING_PWM_TRESHOLD      0

#define BDC_ENCODER_GPIO_A            36
#define BDC_ENCODER_GPIO_B            35
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

/*
typedef struct {
    bdc_motor_handle_t	 motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;
*/

void pwm_vdc_motor_control_thread(void);


#endif /* MAIN_DC_MOTOR_CONTROL_H_ */
