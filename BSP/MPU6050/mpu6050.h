#ifndef BSP_MPU6050_H_
#define BSP_MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MPU6050_STATUS_OK = 0,
    MPU6050_STATUS_NO_ACK,
    MPU6050_STATUS_WRONG_ID,
    MPU6050_STATUS_CONFIG_FAILED,
    MPU6050_STATUS_CALIBRATION_FAILED,
    MPU6050_STATUS_RUNTIME_FAILED
} Mpu6050Status;

bool Mpu6050_Init(void);
bool Mpu6050_Calibrate(void);
bool Mpu6050_Update(uint16_t elapsed_ms, bool stationary);
void Mpu6050_ResetRelativeYaw(void);
int32_t Mpu6050_GetYawRateMdps(void);
int32_t Mpu6050_GetYawMdeg(void);
int32_t Mpu6050_GetRelativeYawMdeg(void);
Mpu6050Status Mpu6050_GetStatus(void);

extern volatile uint8_t g_mpu6050_who_am_i;
extern volatile int32_t g_mpu6050_bias_raw;
extern volatile int32_t g_mpu6050_rate_mdps;
extern volatile int32_t g_mpu6050_yaw_mdeg;
extern volatile uint32_t g_mpu6050_error_count;

#endif /* BSP_MPU6050_H_ */
