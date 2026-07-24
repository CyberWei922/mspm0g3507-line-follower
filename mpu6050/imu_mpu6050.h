#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IMU_MPU6050_OK = 0,
    IMU_MPU6050_ID_READ_FAILED,
    IMU_MPU6050_WRONG_ID,
    IMU_MPU6050_DMP_INIT_FAILED,
    IMU_MPU6050_SENSITIVITY_FAILED,
    IMU_MPU6050_CALIBRATION_FAILED,
    IMU_MPU6050_NO_DMP_DATA
} ImuMpu6050Status;

typedef enum {
    IMU_DMP_SAMPLE_MISSED = 0,
    IMU_DMP_SAMPLE_OK,
    IMU_DMP_SAMPLE_YAW_JUMP
} ImuDmpSampleStatus;

extern volatile ImuMpu6050Status gImuStatus;
extern volatile ImuDmpSampleStatus gImuLastDmpSampleStatus;
extern volatile uint8_t gImuWhoAmI;
extern volatile uint8_t gImuDmpInitCode;
extern volatile bool gImuYawValid;
extern volatile bool gImuYawFresh;
extern volatile float gImuYawDegrees;
extern volatile float gImuYawUnwrappedDegrees;
extern volatile float gImuGyroZDps;
extern volatile float gImuGyroZBiasDps;
extern volatile uint32_t gImuRawReadErrors;
extern volatile uint32_t gImuDmpReadMisses;
extern volatile uint32_t gImuRejectedYawJumps;

bool IMU_MPU6050_init(void);
bool IMU_MPU6050_restartStream(void);
bool IMU_MPU6050_update(void);
float IMU_MPU6050_relativeAngle(float startYawUnwrapped);
float IMU_MPU6050_signedRelativeAngle(float startYawUnwrapped);

#endif
