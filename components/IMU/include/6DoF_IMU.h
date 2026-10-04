/*
 * 6DoF_IMU.h
 *
 *  Created on: 7 wrze 2026
 *      Author: ksapi
 */
 
#ifndef COMPONENTS_6DOF_IMU_H_
#define COMPONENTS_6DOF_IMU_H_

 
#include "DC_motor_control.h"
#include "math.h"


#define LSM6DSO32_SPI_HOST    SPI2_HOST
#define PIN_NUM_MISO          19
#define PIN_NUM_MOSI          23
#define PIN_NUM_CLK           18
#define PIN_NUM_CS            5
#define OUTX_L_G_REG 0x22
#define DEG2RAD (M_PI / 180.0f)
#define ALPHA 0.98f
#define DT 0.002f




 typedef struct {
	float elevation_rad;
	float elevation_rads;
	float horizontal_rad;
	float horizontal_rads;
	motor_control_context_t elevation_motor;
	motor_control_context_t horizontal_motor;
 } system_state_t;
 
 
void vImuTask(void *pvParameters);

#endif /* COMPONENTS_6DOF_IMU_H_ */