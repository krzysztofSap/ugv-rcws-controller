/*
 * PID_controller.h
 *
 *  Created on: 16 wrz 2026
 *      Author: ksapi
 */

#ifndef COMPONENTS_PID_CONTROLLER_H_
#define COMPONENTS_PID_CONTROLLER_H_

#include "6DoF_IMU.h"
#include <stdbool.h>

#define DT 0.002f
#define PID_KP 1.0f
#define PID_KI 0.1f
#define PID_KD 0.1f
#define PID_OUT_MIN -400
#define PID_OUT_MAX 400



typedef struct {
	float elevation_sp;	 // elevation angle setpoint
	float horizontal_sp; // horizontal angle setpoint
    float e[6];		   	 // error  memory, elevation: e[0] = e(k), e[1] = e(k-1), e[2] = e(k-2), horizontal: e[3] = e(k), e[4] = e(k-1), e[5] = e(k-2)
    float prev_output[2];   // last output u(k-1), elevation: prev_output[0] = elevation u(k-1), prev_output[1] = horizontal u(k-1)
	bool estop;
} pid_controller_t;

typedef struct {
	system_state_t *system_state;
	pid_controller_t *pid;
} module_info_t;


void vPIDTask(void *pvParameters);

#endif /* COMPONENTS_PID_CONTROLLER_H_ */
