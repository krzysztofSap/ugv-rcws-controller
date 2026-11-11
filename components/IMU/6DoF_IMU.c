/*
 * 6DoF_IMU.c  (wersja I2C)
 *
 *  Created on: 7 wrze 2026
 *      Author: ksapi
 *
 *  LSM6DSO32 po I2C (Adafruit: CS -> 3V3, DO -> GND => adres 0x6A).
 */

#include "6DoF_IMU.h"
#include "DC_motor_control.h"
#include "driver/i2c_master.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char *TAG_IMU = "IMU";

/*********************************************************/
/*                 KONFIGURACJA (dostosuj)               */
/*********************************************************/



typedef struct {
    float fused_yaw;
    float gyro_bias;
} yaw_fusion_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} imu_i2c_t;

/*********************************************************/
/*                    I2C HELPERS                        */
/*********************************************************/

static esp_err_t imu_write_reg(imu_i2c_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(i2c->dev, buf, sizeof(buf), IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_read_regs(imu_i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(i2c->dev, &reg, 1, data, len, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_i2c_init(imu_i2c_t *i2c) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA,
        .scl_io_num = IMU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   // plytka Adafruit ma wlasne pull-upy
    };
    ESP_LOGI(TAG_IMU, "I2C bus initialization");
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c->bus);
    if (err != ESP_OK) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    ESP_LOGI(TAG_IMU, "I2C add device 0x%02X", IMU_I2C_ADDR);
    return i2c_master_bus_add_device(i2c->bus, &dev_cfg, &i2c->dev);
}

static esp_err_t imu_configure(imu_i2c_t *i2c) {
    esp_err_t err;
    uint8_t v = 0;

    // Software reset i oczekiwanie na zakonczenie
    imu_write_reg(i2c, REG_CTRL3_C, CTRL3_C_SW_RESET);
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (imu_read_regs(i2c, REG_CTRL3_C, &v, 1) == ESP_OK && !(v & CTRL3_C_SW_RESET)) break;
    }

    err = imu_read_regs(i2c, REG_WHO_AM_I, &v, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_IMU, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG_IMU, "WHO_AM_I = 0x%02X (expected 0x%02X)", v, WHO_AM_I_VALUE);
    if (v != WHO_AM_I_VALUE) return ESP_ERR_NOT_FOUND;

    // Uwaga: NIE ustawiamy I2C_disable w CTRL4_C (odcialoby I2C)
    if ((err = imu_write_reg(i2c, REG_CTRL3_C, CTRL3_C_BDU_IF_INC)) != ESP_OK) return err;
    if ((err = imu_write_reg(i2c, REG_CTRL1_XL, CTRL1_XL_833HZ_4G)) != ESP_OK) return err;
    if ((err = imu_write_reg(i2c, REG_CTRL2_G, CTRL2_G_833HZ_125DPS)) != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));  // ustalenie sie filtrow po wlaczeniu czujnika
    return ESP_OK;
}

/*********************************************************/
/*                  IMU HELPER FUNCTIONS                 */
/*********************************************************/

typedef struct {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
} imu_raw_t;

static esp_err_t imu_read_raw(imu_i2c_t *i2c, imu_raw_t *raw) {
    uint8_t d[12];
    esp_err_t err = imu_read_regs(i2c, REG_OUTX_L_G, d, sizeof(d));
    if (err != ESP_OK) return err;

    // Kolejnosc w rejestrach: G(x,y,z), XL(x,y,z); little endian
    raw->gx = (int16_t)((d[1]  << 8) | d[0]);
    raw->gy = (int16_t)((d[3]  << 8) | d[2]);
    raw->gz = (int16_t)((d[5]  << 8) | d[4]);
    raw->ax = (int16_t)((d[7]  << 8) | d[6]);
    raw->ay = (int16_t)((d[9]  << 8) | d[8]);
    raw->az = (int16_t)((d[11] << 8) | d[10]);
    return ESP_OK;
}

static float normalize_angle(float angle) {
    if (!isfinite(angle)) return 0.0f;      // ochrona przed NaN/Inf (petla while by sie zawiesila)
    angle = fmodf(angle + IMU_PI_F, 2.0f * IMU_PI_F);
    if (angle < 0.0f) angle += 2.0f * IMU_PI_F;
    return angle - IMU_PI_F;
}

static float angle_difference(float target, float source) {
    return normalize_angle(target - source);
}

static float pitch_from_accel(const imu_raw_t *r) {
    // float zamiast int: ay*ay + az*az moze przepelnic int32
    float ax = (float)r->ax, ay = (float)r->ay, az = (float)r->az;
    return atan2f(-ax, sqrtf(ay * ay + az * az));
}

static void update_yaw_fusion(yaw_fusion_t *state, float encoder_rad, float gz_rads, float dt) {
    // calkowanie predkosci katowej po odjeciu biasu
    float corrected_gz = gz_rads - state->gyro_bias;
    state->fused_yaw = normalize_angle(state->fused_yaw + corrected_gz * dt);

    // roznica miedzy katem z calkowania a katem z enkodera
    float error = angle_difference(encoder_rad, state->fused_yaw);

    // filtr komplementarny
    state->fused_yaw = normalize_angle(state->fused_yaw + (1.0f - ALPHA) * error);

    // adaptacja biasu zyroskopu
    const float bias_learning_rate = 0.05f;
    state->gyro_bias -= error * bias_learning_rate * dt;
}

static bool read_encoder_rad(system_state_t *s, float *out_rad) {
    int count = 0;
    if (pcnt_unit_get_count(s->horizontal_motor.pcnt_encoder, &count) != ESP_OK) return false;
    *out_rad = normalize_angle((float)count * (2.0f * IMU_PI_F / YAW_ENCODER_COUNTS_PER_REV));
    return true;
}

#if IMU_CALIBRATE_ON_START
static esp_err_t imu_calibrate(imu_i2c_t *i2c, float bias_g[3], float *pitch0) {
    double sum[3] = {0}, sum_pitch = 0;
    int n = 0;
    ESP_LOGI(TAG_IMU, "Gyro calibration - keep the device still...");
    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        imu_raw_t r;
        if (imu_read_raw(i2c, &r) == ESP_OK) {
            sum[0] += r.gx; sum[1] += r.gy; sum[2] += r.gz;
            sum_pitch += pitch_from_accel(&r);
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS > 0 ? IMU_PERIOD_MS : 1) + 1);
    }
    if (n < IMU_CALIB_SAMPLES / 2) return ESP_FAIL;
    for (int k = 0; k < 3; k++) {
        bias_g[k] = (float)(sum[k] / n) * GYRO_DPS_PER_LSB * IMU_DEG2RAD;
    }
    *pitch0 = (float)(sum_pitch / n);
    ESP_LOGI(TAG_IMU, "Gyro bias [rad/s]: %.5f %.5f %.5f", bias_g[0], bias_g[1], bias_g[2]);
    return ESP_OK;
}
#endif

/*********************************************************/
/*                    FREERTOS IMU TASK                  */
/*********************************************************/

void vImuTask(void *pvParameters) {
    if (pvParameters == NULL) {
        ESP_LOGE(TAG_IMU, "system_state == NULL, task stopped");
        vTaskDelete(NULL);
        return;
    }
    system_state_t *system_state = (system_state_t *)pvParameters;

    if (system_state->horizontal_motor.pcnt_encoder == NULL) {
        ESP_LOGE(TAG_IMU, "pcnt_encoder == NULL (encoder not initialised before IMU task)");
        vTaskDelete(NULL);
        return;
    }

    // Okres petli. Przy CONFIG_FREERTOS_HZ=100 pdMS_TO_TICKS(2) == 0 -> petla bez blokowania!
    TickType_t xFrequency = pdMS_TO_TICKS(IMU_PERIOD_MS);
    if (xFrequency == 0) {
        ESP_LOGW(TAG_IMU, "Tick too slow for %d ms period - set CONFIG_FREERTOS_HZ=1000", IMU_PERIOD_MS);
        xFrequency = 1;
    }

    imu_i2c_t i2c = {0};
    esp_err_t err = imu_i2c_init(&i2c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_IMU, "I2C init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    err = imu_configure(&i2c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_IMU, "IMU init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    float bias_g[3] = {0.0f, 0.0f, 0.0f};   // gx, gy, gz [rad/s]
    float pitch0 = 0.0f;
    bool have_pitch0 = false;
#if IMU_CALIBRATE_ON_START
    if (imu_calibrate(&i2c, bias_g, &pitch0) == ESP_OK) {
        have_pitch0 = true;
    } else {
        ESP_LOGW(TAG_IMU, "Calibration failed, using zero bias");
    }
#endif

    // Stan poczatkowy filtrow (bez skoku przy starcie)
    yaw_fusion_t yaw_state = { .fused_yaw = 0.0f, .gyro_bias = 0.0f };
    float enc_rad = 0.0f;
    if (read_encoder_rad(system_state, &enc_rad)) yaw_state.fused_yaw = enc_rad;
    if (have_pitch0) system_state->elevation_rad = pitch0;

    imu_raw_t raw;
    uint32_t err_count = 0;
    int64_t t_prev = esp_timer_get_time();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Rzeczywisty krok czasu (odporny na jitter i zly tick)
        int64_t t_now = esp_timer_get_time();
        float dt = (float)(t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt <= 0.0f || dt > 0.1f) dt = IMU_PERIOD_MS * 1e-3f;

        // Odczyt bez ESP_ERROR_CHECK - blad I2C nie moze abortowac systemu
        err = imu_read_raw(&i2c, &raw);
        if (err != ESP_OK) {
            if ((err_count++ % 100) == 0) {
                ESP_LOGW(TAG_IMU, "I2C read error: %s (count=%lu)", esp_err_to_name(err),
                         (unsigned long)err_count);
            }
            continue;
        }

        // Predkosci katowe [rad/s], po odjeciu biasu z kalibracji
        float gx_rads = raw.gx * GYRO_DPS_PER_LSB * IMU_DEG2RAD - bias_g[0];
        float gy_rads = raw.gy * GYRO_DPS_PER_LSB * IMU_DEG2RAD - bias_g[1];
        float gz_rads = raw.gz * GYRO_DPS_PER_LSB * IMU_DEG2RAD - bias_g[2];
        (void)gx_rads;

        system_state->elevation_rads  = gy_rads;
        system_state->horizontal_rads = gz_rads;

        // Pitch: filtr komplementarny zyroskop + akcelerometr
        float acc_pitch_rad = pitch_from_accel(&raw);
        system_state->elevation_rad =
            ALPHA * (system_state->elevation_rad + gy_rads * dt) + (1.0f - ALPHA) * acc_pitch_rad;

        // Yaw: zyroskop + enkoder
        if (read_encoder_rad(system_state, &enc_rad)) {
            update_yaw_fusion(&yaw_state, enc_rad, gz_rads, dt);
        }
        system_state->horizontal_rad = yaw_state.fused_yaw;
    }
}