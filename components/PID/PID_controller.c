/*
 * PID_controller.c
 *
 *  Created on: 16 wrz 2026
 *      Author: ksapi
 */


#include "PID_controller.h"
#include "6DoF_IMU.h"
#include "DC_motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <stdint.h>


void vPIDTask(void *pvParameters) {
	system_state_t *system_state = (system_state_t *)pvParameters;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(2);
	
	pid_controller_t pid = {
		.elevation_sp = 0,
		.horizontal_sp = 0,
	    .e = {0, 0, 0, 0, 0, 0},		   	
	    .prev_output = {0, 0}
	};
	
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
	    pid.e[0] = pid.elevation_sp - system_state->elevation_rad;
	    //pid.e[0] = pid->is_angular ? normalize_angle(raw_error) : raw_error;
	
	    // incremental PID Terms
	    delta_p = PID_KP * (pid.e[0] - pid.e[1]);
	    delta_i = PID_KI * DT * pid.e[0];
	    delta_d = (PID_KD / DT) * (pid.e[0] - 2.0f * pid.e[1] + pid.e[2]);
	
	    // total change in control signal
	    delta_u = delta_p + delta_i + delta_d;
	
	    // accumulate output: u(k) = u(k-1) + delta_u
	    output =  pid.prev_output[0] + delta_u;
	
	    // output saturation
	    if (output > PID_OUT_MAX) {
	        output = PID_OUT_MAX;
	    } else if (output < PID_OUT_MIN) {
	        output = PID_OUT_MIN;
	    }
	
	    // shift error history and update state
		system_state->elevation_motor.pwm_cmpr_value = output;
	    pid.e[2] = pid.e[1];
	    pid.e[1] = pid.e[0];
	    pid.prev_output[0] = output;
		
		//**************************
		// horizontal axis
		//*************************
		
		// calculate current error
	    pid.e[3] = pid.horizontal_sp - system_state->horizontal_rad;
	    //pid.e[0] = pid->is_angular ? normalize_angle(raw_error) : raw_error;
	
	    // incremental PID Terms
	    delta_p = PID_KP * (pid.e[3] - pid.e[4]);
	    delta_i = PID_KI * DT * pid.e[3];
	    delta_d = (PID_KD / DT) * (pid.e[3] - 2.0f * pid.e[4] + pid.e[5]);
	
	    // total change in control signal
	    delta_u = delta_p + delta_i + delta_d;
	
	    // accumulate output: u(k) = u(k-1) + delta_u
	    output =  pid.prev_output[1] + delta_u;
	
	    // output saturation
	    if (output > PID_OUT_MAX) {
	        output = PID_OUT_MAX;
	    } else if (output < PID_OUT_MIN) {
	        output = PID_OUT_MIN;
	    }
	
	    // shift error history and update state
		system_state->horizontal_motor.pwm_cmpr_value = output;
	    pid.e[5] = pid.e[4];
	    pid.e[4] = pid.e[3];
	    pid.prev_output[1] = output;
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

    }
}
