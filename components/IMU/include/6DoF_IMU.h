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


#ifndef IMU_I2C_PORT
#define IMU_I2C_PORT            I2C_NUM_0
#endif
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA             19     
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL             18
#endif
#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR            0x6A    // DO -> GND = 0x6A
#endif
#ifndef IMU_I2C_FREQ_HZ
#define IMU_I2C_FREQ_HZ         400000
#endif
#define IMU_I2C_TIMEOUT_MS      10

#ifndef IMU_PERIOD_MS
#define IMU_PERIOD_MS           2       // 500 Hz
#endif

#ifndef YAW_ENCODER_COUNTS_PER_REV
#define YAW_ENCODER_COUNTS_PER_REV  8736.0f  
#endif

#ifndef IMU_CALIBRATE_ON_START
#define IMU_CALIBRATE_ON_START  1       // wymaga bezruchu przez ~1 s po starcie
#endif
#define IMU_CALIB_SAMPLES       500

// LSM6DSO32 registers
#define REG_WHO_AM_I            0x0F
#define REG_CTRL1_XL            0x10
#define REG_CTRL2_G             0x11
#define REG_CTRL3_C             0x12
#define REG_OUTX_L_G            0x22
#define WHO_AM_I_VALUE          0x6C

#define CTRL3_C_SW_RESET        0x01
#define CTRL3_C_BDU_IF_INC      0x44    // BDU=1, IF_INC=1
#define CTRL1_XL_833HZ_4G       0x70    // 833 Hz, +/-4 g
#define CTRL2_G_833HZ_125DPS    0x72    // 833 Hz, +/-125 dps (FS_125=1)

#define GYRO_DPS_PER_LSB        0.004375f   // czulosc przy +/-125 dps
#define IMU_PI_F                3.14159265358979f
#define IMU_DEG2RAD             (IMU_PI_F / 180.0f)


#define ALPHA 0.98f


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