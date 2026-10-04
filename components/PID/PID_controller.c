/*
 * PID_controller.c
 *
 *  Created on: 16 wrz 2026
 *      Author: ksapi
 */


#include "PID_controller.h"
#include "DC_motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <stdint.h>



/*********************************************************/
/*                 FREERTOS PID TASK                     */
/*********************************************************/


void vPIDTask(void *pvParameters) {
	module_info_t *module_info = (module_info_t *)pvParameters;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(2);
	
	
	//float raw_error = 0.0;
	float delta_p = 0.0;
	float delta_i = 0.0;
    float delta_d = 0.0;
	float delta_u = 0.0;
	int16_t output = 0;
	
	while(1){
		//**************************
		// elevation axis
		//*************************
		 
	    // calculate current error
	    module_info->pid->e[0] = module_info->pid->elevation_sp - module_info->system_state->elevation_rad;
	    //pid.e[0] = pid->is_angular ? normalize_angle(raw_error) : raw_error;
	
	    // incremental PID Terms
	    delta_p = PID_KP * (module_info->pid->e[0] - module_info->pid->e[1]);
	    delta_i = PID_KI * DT * module_info->pid->e[0];
	    delta_d = (PID_KD / DT) * (module_info->pid->e[0] - 2.0f * module_info->pid->e[1] + module_info->pid->e[2]);
	
	    // total change in control signal
	    delta_u = delta_p + delta_i + delta_d;
	
	    // accumulate output: u(k) = u(k-1) + delta_u
	    output =  module_info->pid->prev_output[0] + delta_u;
	
	    // output saturation
	    if (output > PID_OUT_MAX) {
	        output = PID_OUT_MAX;
	    } else if (output < PID_OUT_MIN) {
	        output = PID_OUT_MIN;
	    }
	
	    // shift error history and update state
		module_info->system_state->elevation_motor.pwm_cmpr_value = output;
	    module_info->pid->e[2] = module_info->pid->e[1];
	    module_info->pid->e[1] = module_info->pid->e[0];
	    module_info->pid->prev_output[0] = output;
		
		//**************************
		// horizontal axis
		//*************************
		
		// calculate current error
	    module_info->pid->e[3] = module_info->pid->horizontal_sp - module_info->system_state->horizontal_rad;
	    //pid.e[0] = pid->is_angular ? normalize_angle(raw_error) : raw_error;
	
	    // incremental PID Terms
	    delta_p = PID_KP * (module_info->pid->e[3] - module_info->pid->e[4]);
	    delta_i = PID_KI * DT * module_info->pid->e[3];
	    delta_d = (PID_KD / DT) * (module_info->pid->e[3] - 2.0f * module_info->pid->e[4] + module_info->pid->e[5]);
	
	    // total change in control signal
	    delta_u = delta_p + delta_i + delta_d;
	
	    // accumulate output: u(k) = u(k-1) + delta_u
	    output =  module_info->pid->prev_output[1] + delta_u;
	
	    // output saturation
	    if (output > PID_OUT_MAX) {
	        output = PID_OUT_MAX;
	    } else if (output < PID_OUT_MIN) {
	        output = PID_OUT_MIN;
	    }
	
	    // shift error history and update state
		module_info->system_state->horizontal_motor.pwm_cmpr_value = output;
	    module_info->pid->e[5] = module_info->pid->e[4];
	    module_info->pid->e[4] = module_info->pid->e[3];
	    module_info->pid->prev_output[1] = output;
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }
}
