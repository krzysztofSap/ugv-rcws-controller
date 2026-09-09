/*
 * 6DoF_IMU.h
 *
 *  Created on: 7 wrze 2026
 *      Author: ksapi
 */
 
#include "DC_motor_control.h"
#define LSM6DSO32_SPI_HOST    SPI2_HOST
#define PIN_NUM_MISO          19
#define PIN_NUM_MOSI          23
#define PIN_NUM_CLK           18
#define PIN_NUM_CS            5
#define OUTX_L_G_REG 0x22
#define DEG2RAD (M_PI / 180.0f)


 typedef struct {
	float elevation_rad;
	float elevation_rads;
	float horizontal_rad;
	float horizontal_rads;
 } system_state_t;
 
 void init_spi_dma(void);
 
 void vImuTask(system_state_t *system_state, encoders_state_t *encoders_state);