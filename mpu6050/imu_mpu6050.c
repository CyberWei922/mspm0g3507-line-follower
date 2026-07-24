#include "imu_mpu6050.h"

#include <math.h>

#include "bsp_mpu6050.h"
#include "delay.h"
#include "inv_mpu.h"

#define MPU6050_ADDRESS              (0x68U)
#define MPU6050_WHO_AM_I_REGISTER    (0x75U)
#define MPU6050_EXPECTED_WHO_AM_I    (0x68U)
#define IMU_DLPF_TARGET_HZ           (42U)
#define IMU_WARMUP_DELAY_MS          (2000UL)
#define IMU_CALIBRATION_SAMPLES      (400U)
#define IMU_CALIBRATION_DELAY_MS     (5UL)
#define IMU_CALIBRATION_MAX_SPAN_DPS (5.0f)
#define IMU_FIRST_DMP_ATTEMPTS       (30U)
#define IMU_DEFAULT_MAX_YAW_STEP_DEGREES (30.0f)
#define IMU_GYRO_DEADBAND_DPS        (0.30f)
#define IMU_RATE_KALMAN_Q            (0.04f)
#define IMU_RATE_KALMAN_R            (1.50f)
#define IMU_STATIONARY_RATE_LIMIT_DPS (0.80f)
#define IMU_STATIONARY_MIN_SAMPLES   (25U)
#define IMU_BIAS_ADAPT_WEIGHT        (0.002f)

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
volatile float gImuGyroZCalibrationSpanDps = 0.0f;
volatile float gImuYawScaleFactor = 1.0f;
volatile float gImuRateKalmanGain = 0.0f;
volatile uint16_t gImuCalibrationSamples = 0U;
volatile uint16_t gImuDLPFHz = 0U;
volatile uint32_t gImuRawReadErrors = 0U;
volatile uint32_t gImuDmpReadMisses = 0U;
volatile uint32_t gImuRejectedYawJumps = 0U;

static float sGyroSensitivity = 16.4f;
static float sPreviousYaw;
static float sGyroHistory[3];
static uint8_t sGyroHistoryCount;
static uint8_t sGyroHistoryIndex;
static float sCalibrationSum;
static float sCalibrationMinimum;
static float sCalibrationMaximum;
static ImuCalibrationState sCalibrationState = IMU_CALIBRATION_IDLE;
static float sKalmanRateEstimate;
static float sKalmanRateCovariance = 1.0f;
static uint16_t sStationarySamples;
static float sMaxYawStepDegrees = IMU_DEFAULT_MAX_YAW_STEP_DEGREES;

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

static void resetRateFilter(void)
{
    sGyroHistoryCount = 0U;
    sGyroHistoryIndex = 0U;
    sKalmanRateEstimate = 0.0f;
    sKalmanRateCovariance = 1.0f;
    sStationarySamples = 0U;
    gImuGyroZDps = 0.0f;
    gImuRateKalmanGain = 0.0f;
}

static bool updateDmpYaw(bool stationaryLocked)
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

        if (absoluteFloat(delta) > sMaxYawStepDegrees) {
            gImuRejectedYawJumps++;
            gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_YAW_JUMP;
            return false;
        }
        /*
         * Scale is measured by a physical 360-degree calibration. While the
         * vehicle is confirmed stationary, consume the packet but suppress
         * its delta so DMP yaw drift cannot accumulate during READY/PAUSED.
         */
        if (!stationaryLocked) {
            gImuYawUnwrappedDegrees += delta * gImuYawScaleFactor;
        }
    }

    gImuYawDegrees = yaw;
    sPreviousYaw = yaw;
    gImuYawFresh = true;
    gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_OK;
    return true;
}

bool IMU_MPU6050_prepare(void)
{
    uint8_t whoAmI = 0U;
    unsigned short lpfHz = 0U;

    gImuStatus = IMU_MPU6050_ID_READ_FAILED;
    gImuYawValid = false;
    gImuYawFresh = false;
    gImuDmpInitCode = 0U;
    gImuLastDmpSampleStatus = IMU_DMP_SAMPLE_MISSED;
    gImuRawReadErrors = 0U;
    gImuDmpReadMisses = 0U;
    gImuRejectedYawJumps = 0U;
    gImuCalibrationSamples = 0U;
    gImuGyroZCalibrationSpanDps = 0.0f;
    gImuDLPFHz = 0U;
    sMaxYawStepDegrees = IMU_DEFAULT_MAX_YAW_STEP_DEGREES;
    sCalibrationState = IMU_CALIBRATION_IDLE;
    resetRateFilter();

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

    /*
     * mpu_set_sample_rate() inside the DMP setup chooses sample_rate/2
     * (20 Hz here). Reapply 42 Hz after DMP initialization and read it back.
     */
    if ((mpu_set_lpf(IMU_DLPF_TARGET_HZ) != 0) ||
        (mpu_get_lpf(&lpfHz) != 0) ||
        (lpfHz != IMU_DLPF_TARGET_HZ)) {
        gImuStatus = IMU_MPU6050_LPF_CONFIG_FAILED;
        return false;
    }
    gImuDLPFHz = lpfHz;
    return true;
}

void IMU_MPU6050_startCalibration(void)
{
    gImuCalibrationSamples = 0U;
    gImuGyroZCalibrationSpanDps = 0.0f;
    sCalibrationSum = 0.0f;
    sCalibrationMinimum = 1000000.0f;
    sCalibrationMaximum = -1000000.0f;
    sCalibrationState = IMU_CALIBRATION_IN_PROGRESS;
}

ImuCalibrationState IMU_MPU6050_calibrationStep(void)
{
    float rateDps;

    if (sCalibrationState != IMU_CALIBRATION_IN_PROGRESS) {
        return sCalibrationState;
    }
    if (!readMountedGyroZ(&rateDps)) {
        gImuStatus = IMU_MPU6050_CALIBRATION_FAILED;
        sCalibrationState = IMU_CALIBRATION_FAILED;
        return sCalibrationState;
    }

    sCalibrationSum += rateDps;
    if (rateDps < sCalibrationMinimum) {
        sCalibrationMinimum = rateDps;
    }
    if (rateDps > sCalibrationMaximum) {
        sCalibrationMaximum = rateDps;
    }
    gImuCalibrationSamples++;

    if (gImuCalibrationSamples >= IMU_CALIBRATION_SAMPLES) {
        gImuGyroZCalibrationSpanDps =
            sCalibrationMaximum - sCalibrationMinimum;
        if (gImuGyroZCalibrationSpanDps >
            IMU_CALIBRATION_MAX_SPAN_DPS) {
            gImuStatus = IMU_MPU6050_CALIBRATION_MOVED;
            sCalibrationState = IMU_CALIBRATION_FAILED;
            return sCalibrationState;
        }
        gImuGyroZBiasDps =
            sCalibrationSum / (float) IMU_CALIBRATION_SAMPLES;
        resetRateFilter();
        sCalibrationState = IMU_CALIBRATION_COMPLETE;
    }
    return sCalibrationState;
}

bool IMU_MPU6050_finishCalibration(void)
{
    uint16_t sample;

    if (sCalibrationState != IMU_CALIBRATION_COMPLETE) {
        gImuStatus = IMU_MPU6050_CALIBRATION_FAILED;
        return false;
    }
    if (mpu_reset_fifo() != 0) {
        gImuStatus = IMU_MPU6050_DMP_INIT_FAILED;
        return false;
    }
    for (sample = 0U; sample < IMU_FIRST_DMP_ATTEMPTS; sample++) {
        delay_ms(10UL);
        if (updateDmpYaw(true)) {
            gImuStatus = IMU_MPU6050_OK;
            return true;
        }
    }

    gImuStatus = IMU_MPU6050_NO_DMP_DATA;
    return false;
}

bool IMU_MPU6050_init(void)
{
    uint16_t sample;

    if (!IMU_MPU6050_prepare()) {
        return false;
    }
    delay_ms(IMU_WARMUP_DELAY_MS);
    IMU_MPU6050_startCalibration();
    for (sample = 0U; sample < IMU_CALIBRATION_SAMPLES; sample++) {
        delay_ms(IMU_CALIBRATION_DELAY_MS);
        if (IMU_MPU6050_calibrationStep() == IMU_CALIBRATION_FAILED) {
            return false;
        }
    }
    return IMU_MPU6050_finishCalibration();
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
        if (updateDmpYaw(true)) {
            gImuStatus = IMU_MPU6050_OK;
            return true;
        }
    }

    gImuStatus = IMU_MPU6050_NO_DMP_DATA;
    return false;
}

void IMU_MPU6050_setYawJumpLimit(float maxStepDegrees)
{
    /*
     * Normal tracking uses a conservative limit. A corner may contain a
     * larger DMP FIFO interval while the chassis is pivoting, so the caller
     * can temporarily raise the limit without disabling jump protection.
     */
    if (maxStepDegrees < IMU_DEFAULT_MAX_YAW_STEP_DEGREES) {
        maxStepDegrees = IMU_DEFAULT_MAX_YAW_STEP_DEGREES;
    } else if (maxStepDegrees > 180.0f) {
        maxStepDegrees = 180.0f;
    }
    sMaxYawStepDegrees = maxStepDegrees;
}

bool IMU_MPU6050_update(bool stationary)
{
    float mountedRateDps;
    float correctedRate;
    float medianRate;
    float kalmanGain;
    bool stationaryLocked = false;

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

    /*
     * Scalar Kalman filter for angular rate. This reduces noise; it is not an
     * absolute-yaw observer and therefore cannot remove yaw drift by itself.
     */
    sKalmanRateCovariance += IMU_RATE_KALMAN_Q;
    kalmanGain = sKalmanRateCovariance /
        (sKalmanRateCovariance + IMU_RATE_KALMAN_R);
    sKalmanRateEstimate +=
        kalmanGain * (medianRate - sKalmanRateEstimate);
    sKalmanRateCovariance =
        (1.0f - kalmanGain) * sKalmanRateCovariance;
    gImuRateKalmanGain = kalmanGain;
    gImuGyroZDps = sKalmanRateEstimate;

    if (stationary &&
        (absoluteFloat(correctedRate) < IMU_STATIONARY_RATE_LIMIT_DPS)) {
        if (sStationarySamples < IMU_STATIONARY_MIN_SAMPLES) {
            sStationarySamples++;
        }
        if (sStationarySamples >= IMU_STATIONARY_MIN_SAMPLES) {
            /*
             * Zero-rate update: adapt bias very slowly and prevent DMP yaw
             * accumulation only in states that already guarantee no motion.
             */
            gImuGyroZBiasDps +=
                correctedRate * IMU_BIAS_ADAPT_WEIGHT;
            stationaryLocked = true;
        }
    } else {
        sStationarySamples = 0U;
    }

    (void) updateDmpYaw(stationaryLocked);
    return true;
}

void IMU_MPU6050_setYawScaleFactor(float scaleFactor)
{
    if ((scaleFactor >= 0.50f) && (scaleFactor <= 1.50f)) {
        gImuYawScaleFactor = scaleFactor;
    }
}

float IMU_MPU6050_relativeAngle(float startYawUnwrapped)
{
    return absoluteFloat(gImuYawUnwrappedDegrees - startYawUnwrapped);
}

float IMU_MPU6050_signedRelativeAngle(float startYawUnwrapped)
{
    return gImuYawUnwrappedDegrees - startYawUnwrapped;
}
