/*
 * 6DoF_IMU.c
 *
 *  Created on: 7 wrze 2026
 *      Author: ksapi
 */

#include "6DoF_IMU.h"
#include "DC_motor_control.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <math.h>

#define ALPHA 0.98f
#define DT 0.002f

spi_device_handle_t spi_handle;
uint8_t *spi_tx_buf;
uint8_t *spi_rx_buf;

void init_spi_dma(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 3,                          // CPOL=1, CPHA=1 (Standard for LSM6DSO)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    // DMA bus initialize
    spi_bus_initialize(LSM6DSO32_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(LSM6DSO32_SPI_HOST, &devcfg, &spi_handle);

    // DMA buffors allocation
    spi_tx_buf = heap_caps_malloc(16, MALLOC_CAP_DMA);
    spi_rx_buf = heap_caps_malloc(16, MALLOC_CAP_DMA);
}


static float normalize_angle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}


static float angle_difference(float target, float source) {
    return normalize_angle(target - source);
}

typedef struct {
    float fused_yaw;
    float gyro_bias;
} yaw_fusion_t;


void update_yaw_fusion(float gz_rads, float encoder_rad, yaw_fusion_t *state) {
    // integrating gz rads to gz rad
    float corrected_gz = gz_rads - state->gyro_bias;
    state->fused_yaw += corrected_gz * DT;
    state->fused_yaw = normalize_angle(state->fused_yaw);

    // calculing difference between integrated angle and encoder angle
    float error = angle_difference(encoder_rad, state->fused_yaw);

    // correcting angle with complementary filter
    state->fused_yaw = normalize_angle(state->fused_yaw + (1.0f - ALPHA) * error);

    // calculing gyro bias
    const float bias_learning_rate = 0.05f; // learning coefficient
    state->gyro_bias -= error * bias_learning_rate * DT;
}


void vImuTask(system_state_t *system_state, encoders_state_t *encoders_state) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
	
	yaw_fusion_t yaw_fusion_state;
	int16_t raw_gx;
	int16_t raw_gy;
    int16_t raw_gz;
    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;
	float gy_rads;
	float acc_pitch_rad;
	
	// initialazing SPI transmission with IMU 
	init_spi_dma();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // MSB first and start adress
        spi_tx_buf[0] = OUTX_L_G_REG | 0x80; 

        spi_transaction_t t = {
            .length = 13 * 8, // 1 adress byte and 8 bytes of data
            .tx_buffer = spi_tx_buf,
            .rx_buffer = spi_rx_buf
        };
        
        // DMA blocking transmision
        spi_device_transmit(spi_handle, &t);

        // Data parsing
        raw_gx = (spi_rx_buf[2] << 8) | spi_rx_buf[1];
        raw_gy = (spi_rx_buf[4] << 8) | spi_rx_buf[3];
        raw_gz = (spi_rx_buf[6] << 8) | spi_rx_buf[5];
        
        raw_ax = (spi_rx_buf[8] << 8) | spi_rx_buf[7];
        raw_ay = (spi_rx_buf[10] << 8) | spi_rx_buf[9];
        raw_az = (spi_rx_buf[12] << 8) | spi_rx_buf[11];

        // Calculing rads (gyro +/- 125 dps -> gyro sensivity 4.375 mdps)
        system_state->elevation_rads = (raw_gx * 0.04375f / 1000.0f) * DEG2RAD;
        gy_rads = (raw_gy * 0.04375f / 1000.0f) * DEG2RAD;
        system_state->horizontal_rads = (raw_gz * 0.04375f / 1000.0f) * DEG2RAD;

        // Calculing pitch angle
        acc_pitch_rad = atan2f(-raw_ax, sqrtf(raw_ay * raw_ay + raw_az * raw_az));
        system_state->elevation_rad = ALPHA * (system_state->elevation_rad + gy_rads * DT) + (1.0f - ALPHA) * acc_pitch_rad;
		
		/* ******************
			TODO:
			Utilize elevation motor encoder in the same way ass horizontal encoder
			to improve pitch angle accuracy
		*/ 
		
		
		// Calculing yaw angle 
		update_yaw_fusion(system_state->horizontal_rads, encoders_state->horizontal_encoder , &yaw_fusion_state );
		system_state->horizontal_rad = yaw_fusion_state.fused_yaw;
    }
}