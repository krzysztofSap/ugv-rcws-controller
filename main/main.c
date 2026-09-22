#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "DC_motor_control.h"
#include "6DoF_IMU.h"
#include "PID_controller.h"
#include "portmacro.h"
#include "esp_log.h"


#define MCPWM_GPIO_A              7
#define ELEVATION_IN_GPIO_RIGTH   1
#define ELEVATION_IN_GPIO_LEFT	  2
#define MCPWM_GPIO_B              15
#define HORIZONTAL_IN_GPIO_RIGTH  3
#define HORIZONTAL_IN_GPIO_LEFT   4

static const char *TAG_MAIN = "MAIN";

void app_main(void){

	static system_state_t system_state = {
		.elevation_rad    = 0,
		.elevation_rads   = 0,
		.horizontal_rad   = 0,
		.horizontal_rads  = 0,
		.elevation_motor  = {		
			.pcnt_encoder = NULL,
			.cmpr_ptr = NULL,
			.pwm_cmpr_value = HOLDING_PWM_TRESHOLD,
			.motor_in_gpio_right = ELEVATION_IN_GPIO_RIGTH,
			.motor_in_gpio_left = ELEVATION_IN_GPIO_LEFT},
		.horizontal_motor = {			
			.pcnt_encoder = NULL,
			.cmpr_ptr = NULL,
			.pwm_cmpr_value = HOLDING_PWM_TRESHOLD,
			.motor_in_gpio_right = ELEVATION_IN_GPIO_RIGTH,
			.motor_in_gpio_left = ELEVATION_IN_GPIO_LEFT}
	};
	
	TaskHandle_t xMotorATaskHandle = NULL;
	TaskHandle_t xMotorBTaskHandle = NULL;
	TaskHandle_t xIMUTaskHandle = NULL;
	TaskHandle_t xPIDTaskHandle = NULL;
	
	motor_mcpwm_init(&system_state.elevation_motor.cmpr_ptr, &system_state.horizontal_motor.cmpr_ptr,
		 MCPWM_GPIO_A, MCPWM_GPIO_B);
	
	BaseType_t result = xTaskCreatePinnedToCore(vMotorControlTask, "MotorA", 3072,
		 &system_state.elevation_motor, 5, &xMotorATaskHandle, 1);
	if (result == pdPASS) {
        ESP_LOGI(TAG_MAIN, "Elevation motor task created.");
	} else {
        ESP_LOGE(TAG_MAIN, "Elevation motor task not created.");
    }
	
	result = xTaskCreatePinnedToCore(vMotorControlTask, "MotorB", 3072,
	 		 &system_state.horizontal_motor, 5, &xMotorBTaskHandle, 1);
 	if (result == pdPASS) {
 	 	ESP_LOGI(TAG_MAIN, "Horizontal motor task created.");
    } else {
        ESP_LOGE(TAG_MAIN, "Horizontal motor task not created.");
    }
	 
	result = xTaskCreatePinnedToCore(vPIDTask, "PID", 3072,
		 		 &system_state, 4, &xPIDTaskHandle, 1);
 	if (result == pdPASS) {
 	 	ESP_LOGI(TAG_MAIN, "PID task created.");
    } else {
        ESP_LOGE(TAG_MAIN, "PID task not created.");
    }

	result = xTaskCreatePinnedToCore(vImuTask, "IMU", 3072,
		 		 &system_state, 3, &xIMUTaskHandle, 1);
 	if (result == pdPASS) {
 	 	ESP_LOGI(TAG_MAIN, "IMU task created.");
    } else {
        ESP_LOGE(TAG_MAIN, "IMU task not created.");
    }
};


	