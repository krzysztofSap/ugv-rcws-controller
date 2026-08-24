/*
 * DC_motor_control.c
 *
 *  Created on: 24 sie 2026
 *      Author: ksapi
 */

#include "DC_motor_control.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_err.h"
#include "esp_log.h"
static const char *TAG_MOTOR = "DC_motor_control";

 void motor_mcpwm_init(mcpwm_cmpr_handle_t *comp_ptr){
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
	
	ESP_LOGI(TAG_MOTOR, "Create comparator");
	mcpwm_cmpr_handle_t cmpr = NULL;
	mcpwm_comparator_config_t comparator_cfg = {
		.intr_priority = 0,
		.flags.update_cmp_on_tez = true
	};
	ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, &cmpr));
	
	ESP_LOGI(TAG_MOTOR, "Create generator");
	mcpwm_gen_handle_t gen = NULL;
	mcpwm_generator_config_t generator_cfg = {
		.gen_gpio_num = BDC_MCPWM_GPIO_A				// TODO: mechanism of providing correct GPIO number
	};
	ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_cfg, &gen));
	
	// set the initial compare value
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, BDC_HOLDING_PWM_TRESHOLD));

    ESP_LOGI(TAG_MOTOR, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG_MOTOR, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
	
 }
 
 void pwm_vdc_motor_control_thread(void);
