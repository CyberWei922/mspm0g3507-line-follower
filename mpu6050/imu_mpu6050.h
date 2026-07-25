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
    IMU_MPU6050_LPF_CONFIG_FAILED,
    IMU_MPU6050_CALIBRATION_FAILED,
    IMU_MPU6050_CALIBRATION_MOVED,
    IMU_MPU6050_NO_DMP_DATA
} ImuMpu6050Status;

typedef enum {
    IMU_CALIBRATION_IDLE = 0,
    IMU_CALIBRATION_IN_PROGRESS,
    IMU_CALIBRATION_COMPLETE,
    IMU_CALIBRATION_FAILED
} ImuCalibrationState;

typedef enum {
    IMU_DMP_SAMPLE_MISSED = 0,
    IMU_DMP_SAMPLE_OK,
    IMU_DMP_SAMPLE_YAW_JUMP,
    IMU_DMP_SAMPLE_REANCHORED
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
extern volatile float gImuRelativeGyroAngleDegrees;
extern volatile float gImuGyroZCalibrationSpanDps;
extern volatile float gImuYawScaleFactor;
extern volatile float gImuRateKalmanGain;
extern volatile uint16_t gImuCalibrationSamples;
extern volatile uint16_t gImuDLPFHz;
extern volatile uint32_t gImuRawReadErrors;
extern volatile uint32_t gImuDmpReadMisses;
extern volatile uint32_t gImuRejectedYawJumps;
extern volatile uint32_t gImuCornerYawReanchors;

bool IMU_MPU6050_init(void);
bool IMU_MPU6050_prepare(void);
void IMU_MPU6050_startCalibration(void);
ImuCalibrationState IMU_MPU6050_calibrationStep(void);
bool IMU_MPU6050_finishCalibration(void);
bool IMU_MPU6050_restartStream(void);
bool IMU_MPU6050_update(bool stationary);
void IMU_MPU6050_setCornerMode(bool enabled);
void IMU_MPU6050_setYawJumpLimit(float maxStepDegrees);
void IMU_MPU6050_setYawScaleFactor(float scaleFactor);
void IMU_MPU6050_resetRelativeGyroAngle(void);
float IMU_MPU6050_signedRelativeGyroAngle(void);
float IMU_MPU6050_relativeAngle(float startYawUnwrapped);
float IMU_MPU6050_signedRelativeAngle(float startYawUnwrapped);

#endif
