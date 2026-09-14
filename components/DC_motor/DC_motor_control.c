/*
 * DC_motor_control.c
 *
 *  Created on: 24 sie 2026
 *      Author: ksapi
 */

#include "DC_motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include <sys/types.h>
static const char *TAG_MOTOR = "DC_motor_control";

//******************************************
// SETUP FUNCTION
//******************************************


 void motor_mcpwm_init(mcpwm_cmpr_handle_t *cmpr_A_ptr, mcpwm_cmpr_handle_t *cmpr_B_ptr, u_int8_t GPIO_wave_A, u_int8_t GPIO_wave_B){
	ESP_LOGI(TAG_MOTOR, "Create timer");
	mcpwm_timer_handle_t timer = NULL;
	mcpwm_timer_config_t timer_cfg = {
		.group_id = 0,
		.intr_priority = 0,
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
		.resolution_hz = BDC_MCPWM_FREQ_HZ,
		.period_ticks = BDC_MCPWM_TIMER_RESOLUTION_HZ
	};
	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));
	
	ESP_LOGI(TAG_MOTOR, "Create operator");
	mcpwm_oper_handle_t oper = NULL;
	mcpwm_operator_config_t operator_cfg = {
		.group_id = 0, 	// operator must be in the same group to the timer
		.intr_priority = 0
	};
	ESP_ERROR_CHECK(mcpwm_new_operator(&operator_cfg, &oper));
	
	ESP_LOGI(TAG_MOTOR, "Connect operator to timer");
	ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
	
	ESP_LOGI(TAG_MOTOR, "Create generator for wave A");
	mcpwm_gen_handle_t genA = NULL;
	mcpwm_generator_config_t generator_A_cfg = {
		.gen_gpio_num = GPIO_wave_A
	};
	ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_A_cfg, &genA));
	
	ESP_LOGI(TAG_MOTOR, "Create generator for wave B");
		mcpwm_gen_handle_t genB = NULL;
		mcpwm_generator_config_t generator_B_cfg = {
			.gen_gpio_num = GPIO_wave_B
		};
		ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_B_cfg, &genB));
	
	ESP_LOGI(TAG_MOTOR, "Create comparator A");
	mcpwm_comparator_config_t comparator_cfg = {
		.intr_priority = 0,
		.flags.update_cmp_on_tez = true
	};
	ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, cmpr_A_ptr));
	
	ESP_LOGI(TAG_MOTOR, "Create comparator B");
	ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, cmpr_B_ptr));

	// set the initial compare value
	ESP_LOGI(TAG_MOTOR, "Set initial A comparator value");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*cmpr_A_ptr, BDC_HOLDING_PWM_TRESHOLD));
	
	ESP_LOGI(TAG_MOTOR, "Set initial B comparator value");
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*cmpr_B_ptr, BDC_HOLDING_PWM_TRESHOLD));

    ESP_LOGI(TAG_MOTOR, "Set generator action on timer and compare event wave A");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genA,
                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genA,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmpr_A_ptr, MCPWM_GEN_ACTION_LOW)));

	ESP_LOGI(TAG_MOTOR, "Set generator action on timer and compare event wave B");
	// go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genB,
                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genB,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmpr_B_ptr, MCPWM_GEN_ACTION_LOW)));
																
    ESP_LOGI(TAG_MOTOR, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
	
 }
 
 //******************************************
 // END SETUP
 //******************************************


 
 void vMotorControlTask(motor_control_context_t *motor,  pid_context_t *pid_context, u_int8_t GPIO_wave_A, u_int8_t GPIO_wave_B){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(2);
	mcpwm_cmpr_handle_t *cmpr_A_ptr = NULL;
	mcpwm_cmpr_handle_t *cmpr_B_ptr = NULL;
	
	ESP_LOGI(TAG_MOTOR,"Motor MCPWM init");
	motor_mcpwm_init(cmpr_A_ptr, cmpr_B_ptr, GPIO_wave_A, GPIO_wave_B);
	
	while(1){
		// refreshing camparators value
		if (motor->pwm_cmpr_value >= 0) { // motor turns right 
			mcpwm_comparator_set_compare_value(*cmpr_A_ptr, motor->pwm_cmpr_value);
			mcpwm_comparator_set_compare_value(*cmpr_B_ptr, 0);
		} 
		else { // motor turns left
			mcpwm_comparator_set_compare_value(*cmpr_B_ptr, motor->pwm_cmpr_value);
			mcpwm_comparator_set_compare_value(*cmpr_A_ptr, 0);
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	
 };
