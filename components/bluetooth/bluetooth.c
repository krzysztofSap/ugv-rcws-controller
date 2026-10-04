/*
 * bluetooth.c
 *
 *  Created on: 22 wrz 2026
 *      Author: ksapi
 */

#include "bluetooth.h"
#include "PID_controller.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <ctype.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "driver/gpio.h"

#include "time.h"
#include "sys/time.h"



// BLE Connection handle, interface, and status flags
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0xFFFF;
static uint16_t s_tx_attr_handle = 0;
static bool s_is_connected = false;

static bool estop_l = false;
static float elevation_sp_l = 0.0f;
static float horizontal_sp_l = 0.0f;

/*********************************************************/
/*  NORDIC UART SERVICE (NUS) UUID DEFINITIONS (128-bit) */
/*********************************************************/

// Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t nus_service_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// RX Characteristic UUID (Write): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t nus_rx_char_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

// TX Characteristic UUID (Notify): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t nus_tx_char_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

// GATT Table Index Enumeration
enum {
    IDX_SVC,
    IDX_CHAR_TX,
    IDX_CHAR_VAL_TX,
    IDX_CHAR_CFG_TX,
    IDX_CHAR_RX,
    IDX_CHAR_VAL_RX,
    HRS_IDX_NB,
};

static uint16_t nus_handle_table[HRS_IDX_NB];

/*********************************************************/
/*           BLE ADVERTISING CONFIGURATION               */
/*********************************************************/

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(nus_service_uuid128),
    .p_service_uuid      = (uint8_t *)nus_service_uuid128,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/*********************************************************/
/*              GATT DATABASE DEFINITION                 */
/*********************************************************/

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Service Declaration
    [IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_PRI_SERVICE}, ESP_GATT_PERM_READ,
    sizeof(nus_service_uuid128), sizeof(nus_service_uuid128), (uint8_t *)nus_service_uuid128}},

    // TX Characteristic Declaration (Notify)
    [IDX_CHAR_TX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ,
    sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    // TX Characteristic Value
    [IDX_CHAR_VAL_TX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)nus_tx_char_uuid128, ESP_GATT_PERM_READ,
    512, 0, NULL}},

    // TX Client Characteristic Configuration Descriptor (CCCD)
    [IDX_CHAR_CFG_TX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG}, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), 0, NULL}},

    // RX Characteristic Declaration (Write / Write Without Response)
    [IDX_CHAR_RX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ,
    sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR}}},

    // RX Characteristic Value
    [IDX_CHAR_VAL_RX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)nus_rx_char_uuid128, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    512, 0, NULL}},
};

/**************************************************/
/*            BLUETOOTH HELPER FUNCTIONS          */
/**************************************************/

static void init_led(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(tag, "LED initialization completed");
}

// sends a text string to the connected Bluetooth BLE client via Notification
esp_err_t bt_send_string(const char *str) {
    if (!s_is_connected || s_gatts_if == ESP_GATT_IF_NONE || s_tx_attr_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    // Send BLE Notification on TX characteristic
    return esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id, s_tx_attr_handle,
                                       strlen(str), (uint8_t *)str, false);
}

// parses and processes control commands received from the terminal
static void parse_rcws_command(const char *data, size_t len)
{
    char buf[64];
    size_t copy_len = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
    memcpy(buf, data, copy_len);
    buf[copy_len] = '\0';

    // strip trailing newlines and spaces
    for (int i = copy_len - 1; i >= 0; i--) {
        if (buf[i] == '\r' || buf[i] == '\n' || buf[i] == ' ') {
            buf[i] = '\0';
        } else {
            break;
        }
    }

    if (strlen(buf) == 0) return;

    ESP_LOGI(tag, "Received command: '%s'", buf);

    float p_val, y_val;

    // 1. Emergency Stop: STOP / ESTOP
    if (strcasecmp(buf, "ESTOP") == 0 || strcasecmp(buf, "STOP") == 0) {
        estop_l = true;
        gpio_set_level(LED_PIN, 1); // Indicate ESTOP on onboard LED
        ESP_LOGE(tag, "!!! EMERGENCY STOP ACTIVATED VIA BT BLE !!!");
        bt_send_string("[RCWS] EMERGENCY STOP ACTIVATED!\n");
    }
    // 2. Set both axes: SET <pitch> <yaw> (e.g., SET 15.0 -30.5)
    else if (sscanf(buf, "SET %f %f", &p_val, &y_val) == 2 || sscanf(buf, "SET:%f,%f", &p_val, &y_val) == 2) {
        elevation_sp_l = p_val;
        horizontal_sp_l = y_val;
        estop_l = false;
        gpio_set_level(LED_PIN, 0);

        char resp[64];
        snprintf(resp, sizeof(resp), "[RCWS] Setpoint: Pitch=%.2f, Yaw=%.2f\n", p_val, y_val);
        bt_send_string(resp);
        ESP_LOGI(tag, "New Setpoint -> Pitch: %.2f | Yaw: %.2f", p_val, y_val);
    }
    // 3. Set Pitch axis only: P <pitch> (e.g., P 12.0)
    else if (sscanf(buf, "P %f", &p_val) == 1 || sscanf(buf, "P:%f", &p_val) == 1) {
        elevation_sp_l = p_val;
        estop_l = false;

        char resp[48];
        snprintf(resp, sizeof(resp), "[RCWS] Setpoint: Pitch=%.2f\n", p_val);
        bt_send_string(resp);
    }
    // 4. Set Yaw axis only: Y <yaw> (e.g., Y -45.0)
    else if (sscanf(buf, "Y %f", &y_val) == 1 || sscanf(buf, "Y:%f", &y_val) == 1) {
        horizontal_sp_l = y_val;
        estop_l = false;

        char resp[48];
        snprintf(resp, sizeof(resp), "[RCWS] Setpoint: Yaw=%.2f\n", y_val);
        bt_send_string(resp);
    }
    else {
        bt_send_string("[RCWS] ERR: Unknown command. Syntax: 'SET <P> <Y>', 'P <deg>', 'Y <deg>', or 'ESTOP'\n");
    }
}

/*********************************************************/
/*              BLE GAP & GATTS CALLBACKS                */
/*********************************************************/

static void esp_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(tag, "BLE Advertising start failed");
        } else {
            ESP_LOGI(tag, "BLE Advertising started successfully");
        }
        break;
    default:
        break;
    }
}

static void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(tag, "ESP_GATTS_REG_EVT");
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            memcpy(nus_handle_table, param->add_attr_tab.handles, sizeof(nus_handle_table));
            s_tx_attr_handle = nus_handle_table[IDX_CHAR_VAL_TX];
            esp_ble_gatts_start_service(nus_handle_table[IDX_SVC]);
            ESP_LOGI(tag, "GATT Attribute Table created successfully");
        } else {
            ESP_LOGE(tag, "Create GATT attribute table failed, error code = 0x%x", param->add_attr_tab.status);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        s_gatts_if = gatts_if;
        s_is_connected = true;
        ESP_LOGI(tag, "BLE Client connected, conn_id = %d", s_conn_id);

        // Welcome banner and command syntax sent to connected BLE terminal
        bt_send_string("\n=== RCWS WEAPON STATION CONNECTED (BLE) ===\n");
        bt_send_string("Commands: SET <pitch> <yaw> | P <pitch> | Y <yaw> | ESTOP\n\n");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(tag, "BLE Client disconnected, reason = 0x%x", param->disconnect.reason);
        s_is_connected = false;
        s_conn_id = 0xFFFF;
        // Restart advertising upon disconnect
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep) {
            // Handle incoming command from RX Characteristic
            if (nus_handle_table[IDX_CHAR_VAL_RX] == param->write.handle) {
                parse_rcws_command((const char *)param->write.value, param->write.len);
            }
        }
        break;

    default:
        break;
    }
}

/*********************************************************/
/*                  FREERTOS BLE TASK                    */
/*********************************************************/

// Background task sending real-time RCWS position and status to the BT BLE terminal
void xBtTask(void *pvParameters)
{
    module_info_t *module_info = (module_info_t *)pvParameters;
    init_led();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release Classic BT memory, enable BLE controller
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(tag, "%s Controller initialization failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(tag, "%s Controller enable failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(tag, "%s Bluedroid initialization failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(tag, "%s Bluedroid enable failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    // Register BLE callbacks
    if ((ret = esp_ble_gatts_register_callback(esp_gatts_cb)) != ESP_OK) {
        ESP_LOGE(tag, "%s GATTS register failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_ble_gap_register_callback(esp_ble_gap_cb)) != ESP_OK) {
        ESP_LOGE(tag, "%s GAP register failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    if ((ret = esp_ble_gatts_app_register(PROFILE_APP_ID)) != ESP_OK) {
        ESP_LOGE(tag, "%s GATTS app register failed: %s\n", __func__, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Transmit every 500 ms

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        module_info->pid->elevation_sp = elevation_sp_l;
        module_info->pid->horizontal_sp = horizontal_sp_l;
        module_info->pid->estop = estop_l;

        if (s_is_connected) {
            char telemetry_buf[128];
            snprintf(telemetry_buf, sizeof(telemetry_buf),
                     "[STAT] P_cur:%.2f P_tgt:%.2f | Y_cur:%.2f Y_tgt:%.2f | ESTOP:%d\n",
                     module_info->system_state->elevation_rad, module_info->pid->elevation_sp,
                     module_info->system_state->horizontal_rad, module_info->pid->horizontal_sp,
                     module_info->pid->estop ? 1 : 0);

            bt_send_string(telemetry_buf);
        }
    }
}