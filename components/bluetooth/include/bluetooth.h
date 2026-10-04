/*
 * bluetooth.h
 *
 *  Created on: 22 wrz 2026
 *      Author: ksapi
 */

#ifndef COMPONENTS_BLUETOOTH_BLUETOOTH_H_
#define COMPONENTS_BLUETOOTH_BLUETOOTH_H_

#define LED_PIN 			GPIO_NUM_2
#define DEVICE_NAME         "RCWS_TURRET"
#define PROFILE_NUM         1
#define PROFILE_APP_ID      0

static const char *tag = "Bluetooth_RCWS_BLE";

void xBtTask(void *pvParameters);

#endif /* COMPONENTS_BLUETOOTH_BLUETOOTH_H_ */
