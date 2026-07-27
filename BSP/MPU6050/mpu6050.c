#include "mpu6050.h"

#include "app_config.h"
#include "BSP/Common/soft_i2c.h"
#include "BSP/System/system_time.h"
#include "ti_msp_dl_config.h"

#define MPU_ADDRESS_PRIMARY  (0x68U)
#define MPU_ADDRESS_SECONDARY (0x69U)
#define REG_SMPLRT_DIV       (0x19U)
#define REG_CONFIG           (0x1AU)
#define REG_GYRO_CONFIG      (0x1BU)
#define REG_GYRO_ZOUT_H      (0x47U)
#define REG_PWR_MGMT_1       (0x6BU)
#define REG_WHO_AM_I         (0x75U)
#define EXPECTED_WHO_AM_I    (0x68U)
#define GYRO_LSB_PER_DPS     (131L)
#define RUNTIME_FAIL_LIMIT   (5U)

/* 定点标量卡尔曼参数，避免控制主循环依赖软件浮点。 */
#define KALMAN_Q             (2500L)
#define KALMAN_R             (40000L)
#define KALMAN_ONE_Q15       (32768L)

volatile uint8_t g_mpu6050_who_am_i;
volatile int32_t g_mpu6050_bias_raw;
volatile int32_t g_mpu6050_rate_mdps;
volatile int32_t g_mpu6050_yaw_mdeg;
volatile uint32_t g_mpu6050_error_count;

static SoftI2cBus s_imu_bus;
static uint8_t s_mpu_address = MPU_ADDRESS_PRIMARY;
static Mpu6050Status s_status = MPU6050_STATUS_NO_ACK;
static int32_t s_rate_history[3];
static uint8_t s_history_count;
static uint8_t s_history_index;
static int32_t s_kalman_estimate;
static int32_t s_kalman_covariance = KALMAN_R;
static int32_t s_relative_start_mdeg;
static uint16_t s_stationary_ms;
static uint8_t s_runtime_failures;

static int32_t absolute_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t median3(int32_t a, int32_t b, int32_t c)
{
    int32_t temporary;

    if (a > b) { temporary = a; a = b; b = temporary; }
    if (b > c) { temporary = b; b = c; c = temporary; }
    if (a > b) { b = a; }
    return b;
}

static bool write_u8(uint8_t reg, uint8_t value)
{
    return SoftI2c_WriteRegister(&s_imu_bus, s_mpu_address, reg, &value, 1U);
}

static bool read_bytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    bool ok = SoftI2c_ReadRegister(&s_imu_bus, s_mpu_address, reg,
        data, length);

    if (!ok) {
        ++g_mpu6050_error_count;
    }
    return ok;
}

static bool read_gyro_z_raw(int16_t *raw)
{
    uint8_t bytes[2];

    if (!read_bytes(REG_GYRO_ZOUT_H, bytes, 2U)) {
        return false;
    }
    *raw = (int16_t) (((uint16_t) bytes[0] << 8U) | bytes[1]);
    return true;
}

static void reset_filter(void)
{
    s_history_count = 0U;
    s_history_index = 0U;
    s_kalman_estimate = 0;
    s_kalman_covariance = KALMAN_R;
    g_mpu6050_rate_mdps = 0;
}

bool Mpu6050_Init(void)
{
    uint8_t id = 0U;

    s_imu_bus.port = MPU_GPIO_PORT;
    s_imu_bus.scl_pin = MPU_GPIO_IMU_SCL_PIN;
    s_imu_bus.sda_pin = MPU_GPIO_IMU_SDA_PIN;
    s_imu_bus.scl_iomux = MPU_GPIO_IMU_SCL_IOMUX;
    s_imu_bus.sda_iomux = MPU_GPIO_IMU_SDA_IOMUX;
    s_imu_bus.half_period_cycles = CPUCLK_FREQ / 166666U;
    s_imu_bus.stretch_timeout_cycles = CPUCLK_FREQ / 1000U;
    SoftI2c_Init(&s_imu_bus);

    g_mpu6050_error_count = 0U;
    g_mpu6050_bias_raw = 0;
    g_mpu6050_yaw_mdeg = 0;
    s_relative_start_mdeg = 0;
    s_stationary_ms = 0U;
    s_runtime_failures = 0U;
    reset_filter();

    /* AD0接地时为0x68；若模块AD0被拉高，也兼容0x69。 */
    s_mpu_address = MPU_ADDRESS_PRIMARY;
    if (!read_bytes(REG_WHO_AM_I, &id, 1U) &&
        (SoftI2c_Probe(&s_imu_bus, MPU_ADDRESS_SECONDARY))) {
        s_mpu_address = MPU_ADDRESS_SECONDARY;
        if (!read_bytes(REG_WHO_AM_I, &id, 1U)) {
            s_status = MPU6050_STATUS_NO_ACK;
            return false;
        }
    } else if (g_mpu6050_error_count != 0U && id == 0U) {
        s_status = MPU6050_STATUS_NO_ACK;
        return false;
    }
    g_mpu6050_who_am_i = id;
    if (id != EXPECTED_WHO_AM_I) {
        s_status = MPU6050_STATUS_WRONG_ID;
        return false;
    }

    if (!write_u8(REG_PWR_MGMT_1, 0x80U)) {
        s_status = MPU6050_STATUS_CONFIG_FAILED;
        return false;
    }
    SystemTime_DelayMs(100U);
    if (!write_u8(REG_PWR_MGMT_1, 0x01U) ||
        !write_u8(REG_CONFIG, 0x03U) ||       /* 约42 Hz DLPF */
        !write_u8(REG_SMPLRT_DIV, 0x04U) ||  /* 1 kHz/(1+4)=200 Hz */
        !write_u8(REG_GYRO_CONFIG, 0x00U)) { /* ±250 dps */
        s_status = MPU6050_STATUS_CONFIG_FAILED;
        return false;
    }

    s_status = MPU6050_STATUS_OK;
    return true;
}

bool Mpu6050_Calibrate(void)
{
    int64_t sum = 0;
    uint16_t good = 0U;
    uint16_t sample;

    if (s_status != MPU6050_STATUS_OK) {
        return false;
    }
    SystemTime_DelayMs(IMU_WARMUP_MS);
    for (sample = 0U; sample < IMU_CALIBRATION_SAMPLES; ++sample) {
        int16_t raw;

        if (read_gyro_z_raw(&raw)) {
            sum += raw;
            ++good;
        }
        SystemTime_DelayMs(IMU_CALIBRATION_PERIOD_MS);
    }
    if (good < (IMU_CALIBRATION_SAMPLES * 3U) / 4U) {
        s_status = MPU6050_STATUS_CALIBRATION_FAILED;
        return false;
    }
    g_mpu6050_bias_raw = (int32_t) (sum / good);
    g_mpu6050_yaw_mdeg = 0;
    s_relative_start_mdeg = 0;
    reset_filter();
    return true;
}

bool Mpu6050_Update(uint16_t elapsed_ms, bool stationary)
{
    int16_t raw;
    int32_t measurement;
    int32_t filtered_measurement;
    int32_t gain_q15;

    if (s_status != MPU6050_STATUS_OK) {
        return false;
    }
    if (!read_gyro_z_raw(&raw)) {
        if (++s_runtime_failures >= RUNTIME_FAIL_LIMIT) {
            s_status = MPU6050_STATUS_RUNTIME_FAILED;
        }
        return false;
    }
    s_runtime_failures = 0U;

    measurement = ((int32_t) raw - g_mpu6050_bias_raw) * 1000L /
        GYRO_LSB_PER_DPS;
    measurement *= IMU_YAW_SIGN;
    s_rate_history[s_history_index] = measurement;
    s_history_index = (uint8_t) ((s_history_index + 1U) % 3U);
    if (s_history_count < 3U) {
        ++s_history_count;
    }
    filtered_measurement = (s_history_count == 3U) ?
        median3(s_rate_history[0], s_rate_history[1], s_rate_history[2]) :
        measurement;
    if (absolute_i32(filtered_measurement) < IMU_RATE_DEADBAND_MDPS) {
        filtered_measurement = 0;
    }

    s_kalman_covariance += KALMAN_Q;
    gain_q15 = (int32_t) (((int64_t) s_kalman_covariance << 15) /
        (s_kalman_covariance + KALMAN_R));
    s_kalman_estimate += (int32_t) (((int64_t) gain_q15 *
        (filtered_measurement - s_kalman_estimate)) >> 15);
    s_kalman_covariance = (int32_t) (((int64_t)
        (KALMAN_ONE_Q15 - gain_q15) * s_kalman_covariance) >> 15);
    g_mpu6050_rate_mdps = s_kalman_estimate;

    g_mpu6050_yaw_mdeg += (int32_t) (((int64_t) g_mpu6050_rate_mdps *
        elapsed_ms * IMU_YAW_SCALE_NUM) /
        (1000LL * IMU_YAW_SCALE_DEN));

    if (stationary &&
        (absolute_i32(g_mpu6050_rate_mdps) < IMU_STATIONARY_RATE_MDPS)) {
        if (s_stationary_ms < IMU_STATIONARY_CONFIRM_MS) {
            s_stationary_ms = (uint16_t) (s_stationary_ms + elapsed_ms);
        }
        if ((s_stationary_ms >= IMU_STATIONARY_CONFIRM_MS) &&
            (absolute_i32(g_mpu6050_yaw_mdeg) <=
                IMU_ZERO_PULL_LIMIT_MDEG)) {
            /* 约每毫秒衰减0.1%，20 ms周期近似衰减2%。 */
            g_mpu6050_yaw_mdeg =
                (int32_t) (((int64_t) g_mpu6050_yaw_mdeg *
                    (1000 - elapsed_ms)) / 1000);
        }
    } else {
        s_stationary_ms = 0U;
    }
    return true;
}

void Mpu6050_ResetRelativeYaw(void)
{
    s_relative_start_mdeg = g_mpu6050_yaw_mdeg;
}

int32_t Mpu6050_GetYawRateMdps(void)
{
    return g_mpu6050_rate_mdps;
}

int32_t Mpu6050_GetYawMdeg(void)
{
    return g_mpu6050_yaw_mdeg;
}

int32_t Mpu6050_GetRelativeYawMdeg(void)
{
    return g_mpu6050_yaw_mdeg - s_relative_start_mdeg;
}

Mpu6050Status Mpu6050_GetStatus(void)
{
    return s_status;
}
