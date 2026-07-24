#include "imu_mpu6050.h"

#include <math.h>

#include "bsp_mpu6050.h"
#include "delay.h"
#include "inv_mpu.h"

#define MPU6050_ADDRESS              (0x68U)
#define MPU6050_WHO_AM_I_REGISTER    (0x75U)
#define MPU6050_EXPECTED_WHO_AM_I    (0x68U)
#define IMU_CALIBRATION_DISCARD      (10U)
#define IMU_CALIBRATION_SAMPLES      (80U)
#define IMU_CALIBRATION_DELAY_MS     (5UL)
#define IMU_FIRST_DMP_ATTEMPTS       (30U)
#define IMU_MAX_YAW_STEP_DEGREES     (30.0f)
#define IMU_GYRO_DEADBAND_DPS        (0.30f)
#define IMU_GYRO_FILTER_NEW_WEIGHT   (0.35f)

volatile ImuMpu6050Status gImuStatus = IMU_MPU6050_ID_READ_FAILED;
volatile ImuDmpSampleStatus gImuLastDmpSampleStatus =
    IMU_DMP_SAMPLE_MISSED;
volatile uint8_t gImuWhoAmI = 0U;
volatile uint8_t gImuDmpInitCode = 0U;
volatile bool gImuYawValid = false;
volatile bool gImuYawFresh = false;
volatile float gImuYawDegrees = 0.0f;
volatile float gImuYawUnwrappedDegrees = 0.0f;
volatile float gImuGyroZDps = 0.0f;
volatile float gImuGyroZBiasDps = 0.0f;
volatile uint32_t gImuRawReadErrors = 0U;
volatile uint32_t gImuDmpReadMisses = 0U;
volatile uint32_t gImuRejectedYawJumps = 0U;

static float sGyroSensitivity = 16.4f;
static float sPreviousYaw;
static float sGyroHistory[3];
static uint8_t sGyroHistoryCount;
static uint8_t sGyroHistoryIndex;

static float absoluteFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float wrapYawDelta(float delta)
{
    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

static float medianOfThree(float a, float b, float c)
{
    if (a > b) {
        float temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c) {
        float temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

static bool readMountedGyroZ(float *mountedRateDps)
{
    short rawGyro[3];
    unsigned long timestamp = 0UL;

    if (mpu_get_gyro_reg(rawGyro, &timestamp) != 0) {
        gImuRawReadErrors++;
        return false;
    }

    /*
     * The module is component-side up, so sensor +Z and vehicle +Z both point
     * upward. The 180-degree in-plane mounting rotation changes X/Y only and
     * does not change yaw-rate sign: left is positive and right is negative.
     */
    *mountedRateDps = (float) rawGyro[2] / sGyroSensitivity;
    return true;
}

static bool updateDmpYaw(void)
{
    float pitch;
    float roll;
    float yaw;

    gImuYawFresh = false;
    gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_MISSED;
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0U) {
        gImuDmpReadMisses++;
        return false;
    }

    if (!gImuYawValid) {
        gImuYawDegrees = yaw;
        gImuYawUnwrappedDegrees = yaw;
        sPreviousYaw = yaw;
        gImuYawValid = true;
        gImuYawFresh = true;
        return true;
    }

    {
        float delta = wrapYawDelta(yaw - sPreviousYaw);

        if (absoluteFloat(delta) > IMU_MAX_YAW_STEP_DEGREES) {
            gImuRejectedYawJumps++;
            gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_YAW_JUMP;
            return false;
        }
        gImuYawUnwrappedDegrees += delta;
    }

    gImuYawDegrees = yaw;
    sPreviousYaw = yaw;
    gImuYawFresh = true;
    gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_OK;
    return true;
}

bool IMU_MPU6050_init(void)
{
    uint8_t whoAmI = 0U;
    uint16_t sample;
    float biasSum = 0.0f;

    gImuStatus = IMU_MPU6050_ID_READ_FAILED;
    gImuYawValid = false;
    gImuYawFresh = false;
    gImuDmpInitCode = 0U;
    gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_MISSED;
    gImuRawReadErrors = 0U;
    gImuDmpReadMisses = 0U;
    gImuRejectedYawJumps = 0U;
    sGyroHistoryCount = 0U;
    sGyroHistoryIndex = 0U;

    if (MPU6050_ReadData(MPU6050_ADDRESS, MPU6050_WHO_AM_I_REGISTER,
            1U, &whoAmI) != 0) {
        return false;
    }

    gImuWhoAmI = whoAmI;
    if (whoAmI != MPU6050_EXPECTED_WHO_AM_I) {
        gImuStatus = IMU_MPU6050_WRONG_ID;
        return false;
    }

    gImuDmpInitCode = mpu_dmp_init();
    if (gImuDmpInitCode != 0U) {
        gImuStatus = IMU_MPU6050_DMP_INIT_FAILED;
        return false;
    }

    if (mpu_get_gyro_sens(&sGyroSensitivity) != 0) {
        gImuStatus = IMU_MPU6050_SENSITIVITY_FAILED;
        return false;
    }

    for (sample = 0U;
         sample < (IMU_CALIBRATION_DISCARD + IMU_CALIBRATION_SAMPLES);
         sample++) {
        float rateDps;

        delay_ms(IMU_CALIBRATION_DELAY_MS);
        if (!readMountedGyroZ(&rateDps)) {
            gImuStatus = IMU_MPU6050_CALIBRATION_FAILED;
            return false;
        }
        if (sample >= IMU_CALIBRATION_DISCARD) {
            biasSum += rateDps;
        }
    }
    gImuGyroZBiasDps = biasSum / (float) IMU_CALIBRATION_SAMPLES;

    if (mpu_reset_fifo() != 0) {
        gImuStatus = IMU_MPU6050_DMP_INIT_FAILED;
        return false;
    }

    for (sample = 0U; sample < IMU_FIRST_DMP_ATTEMPTS; sample++) {
        delay_ms(10UL);
        if (updateDmpYaw()) {
            gImuStatus = IMU_MPU6050_OK;
            return true;
        }
    }

    gImuStatus = IMU_MPU6050_NO_DMP_DATA;
    return false;
}

bool IMU_MPU6050_restartStream(void)
{
    uint16_t attempt;

    gImuYawValid = false;
    gImuYawFresh = false;
    if (mpu_reset_fifo() != 0) {
        gImuStatus = IMU_MPU6050_DMP_INIT_FAILED;
        return false;
    }

    for (attempt = 0U; attempt < 15U; attempt++) {
        delay_ms(10UL);
        if (updateDmpYaw()) {
            gImuStatus = IMU_MPU6050_OK;
            return true;
        }
    }

    gImuStatus = IMU_MPU6050_NO_DMP_DATA;
    return false;
}

bool IMU_MPU6050_update(void)
{
    float mountedRateDps;
    float correctedRate;
    float medianRate;

    gImuYawFresh = false;
    if (!readMountedGyroZ(&mountedRateDps)) {
        return false;
    }

    correctedRate = mountedRateDps - gImuGyroZBiasDps;
    sGyroHistory[sGyroHistoryIndex] = correctedRate;
    sGyroHistoryIndex = (uint8_t) ((sGyroHistoryIndex + 1U) % 3U);
    if (sGyroHistoryCount < 3U) {
        sGyroHistoryCount++;
    }

    if (sGyroHistoryCount == 3U) {
        medianRate = medianOfThree(
            sGyroHistory[0], sGyroHistory[1], sGyroHistory[2]);
    } else {
        medianRate = correctedRate;
    }

    if (absoluteFloat(medianRate) < IMU_GYRO_DEADBAND_DPS) {
        medianRate = 0.0f;
    }
    gImuGyroZDps =
        (IMU_GYRO_FILTER_NEW_WEIGHT * medianRate) +
        ((1.0f - IMU_GYRO_FILTER_NEW_WEIGHT) * gImuGyroZDps);

    (void) updateDmpYaw();
    return true;
}

float IMU_MPU6050_relativeAngle(float startYawUnwrapped)
{
    return absoluteFloat(gImuYawUnwrappedDegrees - startYawUnwrapped);
}

float IMU_MPU6050_signedRelativeAngle(float startYawUnwrapped)
{
    return gImuYawUnwrappedDegrees - startYawUnwrapped;
}
