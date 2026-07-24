/*
 * Yahboom 8-channel black-line PID follower with MPU6050 feedback.
 *
 * Safe behavior:
 *   - power-on/debugger resets command stop and wait
 *   - only a physical RST/NRST press arms the two-second start sequence
 *   - I2C failure, sustained line loss, turn timeout, or sustained
 *     all-black input stops
 *   - confirmed 90-degree corners use forward compensation, pivot, and
 *     center-sensor reacquisition before returning to PID
 *
 * Wiring:
 *   PA12 SCL, PA13 SDA (Yahboom motor driver, software I2C)
 *   PA14 OUT, PA15 AD0, PA16 AD1, PA17 AD2 (8-channel grayscale)
 *   PA1 SCL, PA0 SDA (MPU6050, software I2C; AD0 is wired to GND)
 *   PB24 expansion-board buzzer (TIMA0 CCP3 PWM)
 */

#include "ti_msp_dl_config.h"
#include "firmware_version.h"
#include "imu_mpu6050.h"

#include <stdbool.h>
#include <stdint.h>

/* Kept in the image so CCS/debugger can inspect the exact firmware build. */
const char gFirmwareVersion[] __attribute__((used)) = FIRMWARE_VERSION_TAG;

#define MOTOR_I2C_7BIT_ADDRESS (0x26U)
#define MOTOR_SPEED_REGISTER   (0x06U)

#define BLUE_LED_PIN       (GPIO_LEDS_USER_LED_1_PIN)
#define GREEN_LED_PIN      (GPIO_LEDS_USER_LED_2_PIN)
#define ALL_STATUS_LEDS    (BLUE_LED_PIN | GREEN_LED_PIN)
#define GRAY8_ADDRESS_PINS (GPIO_GRAY8_AD0_PIN | GPIO_GRAY8_AD1_PIN | \
                            GPIO_GRAY8_AD2_PIN)

/*
 * Set this to 0U if the physical installation puts channel 0 on the right.
 * The raised-wheel direction check in README.md must be done before floor use.
 */
#define GRAY8_CHANNEL0_IS_LEFT (1U)

#define MIN_TRACKING_WHEEL_SPEED ((int16_t) 135)
#define MAX_WHEEL_SPEED     ((int16_t) 320)
#define MAX_PID_CORRECTION  ((int16_t) 175)
#define CORNER_APPROACH_SPEED ((int16_t) 210)
#define CORNER_PIVOT_SPEED    ((int16_t) 330)
#define CORNER_PIVOT_SLOW_SPEED ((int16_t) 270)
#define CORNER_SETTLE_SPEED   ((int16_t) 230)
#define CORNER_SETTLE_MAX_CORRECTION ((int16_t) 130)
#define CORNER_CENTER_ERROR_LIMIT ((int16_t) 75)
#define LOST_PIVOT_SPEED      ((int16_t) 210)

#define CORNER_SLOW_ANGLE_DEGREES       (72.0f)
#define CORNER_REACQUIRE_ANGLE_DEGREES  (78.0f)
#define CORNER_ABORT_ANGLE_DEGREES      (145.0f)

/*
 * Positive official pattern error makes the verified chassis turn right.
 * With the actual component-side-up installation, vehicle left yaw is
 * positive and right yaw is negative. The desired yaw rate therefore has the
 * opposite sign to pattern error.
 *
 * The desired yaw rate is low-pass filtered so a one-sensor pattern change
 * cannot instantly reverse the steering demand. Feedback uses
 * (measured - desired), because a positive motor correction turns the chassis
 * right (negative yaw). This is deliberately negative feedback: when the
 * chassis is rotating right too quickly, the added correction becomes
 * negative and counter-steers left.
 */
#define GYRO_TARGET_RATE_PER_ERROR_DPS  (-22.0f)
#define GYRO_TARGET_FILTER_NEW_WEIGHT   (0.45f)
#define GYRO_RATE_FEEDBACK_GAIN         (1.60f)
#define GYRO_CENTER_DAMPING_GAIN        (2.30f)
#define GYRO_RATE_MAX_CORRECTION        ((int16_t) 70)
#define IMU_RAW_FAILURE_LIMIT_CYCLES    (5U)
#define IMU_YAW_STALE_LIMIT_CYCLES      (15U)
#define STARTUP_PLACEMENT_BLINKS        (2U)

/*
 * Yahboom official LineWalking controller constants. V_z is converted to a
 * left/right speed delta using the official 188 mm chassis APB parameter.
 */
#define OFFICIAL_IRR_SPEED          ((int16_t) 320)
#define OFFICIAL_CAR_APB            (188L)
#define OFFICIAL_TURN_KP            (150L)
#define OFFICIAL_TURN_KI            (4L)
#define OFFICIAL_PATTERN_INTEGRAL_LIMIT (80L)
#define OFFICIAL_TURN_KD_NUM        (1L)
#define OFFICIAL_TURN_KD_DEN        (2L)
#define OFFICIAL_MAX_WHEEL_SPEED    ((int16_t) 500)

/* Responsive cloth-track tuning: Kp=0.60, Ki=0, Kd=0.55. */
#define PID_KP_NUM          (60L)
#define PID_KP_DEN          (100L)
#define PID_KI_NUM          (0L)
#define PID_KI_DEN          (10000L)
#define PID_KD_NUM          (55L)
#define PID_KD_DEN          (100L)
#define PID_INTEGRAL_LIMIT  (8000L)

#define LOST_LINE_LIMIT_CYCLES (40U)
#define WIDE_MARK_LIMIT_CYCLES (25U)
#define STARTUP_SAMPLE_COUNT   (32U)
#define CORNER_CONFIRM_CYCLES       (2U)
#define CORNER_APPROACH_CYCLES      (8U)
#define CORNER_REACQUIRE_CYCLES     (4U)
#define CORNER_SETTLE_CYCLES        (8U)
#define CORNER_TURN_TIMEOUT_CYCLES  (120U)
#define LOST_PIVOT_AFTER_CYCLES     (5U)

#define DELAY_5_MS         ((CPUCLK_FREQ / 1000U) * 5U)
#define DELAY_20_MS        ((CPUCLK_FREQ / 1000U) * 20U)
#define DELAY_60_MS        ((CPUCLK_FREQ / 1000U) * 60U)
#define DELAY_90_MS        ((CPUCLK_FREQ / 1000U) * 90U)
#define DELAY_100_MS       (CPUCLK_FREQ / 10U)
#define DELAY_250_MS       (CPUCLK_FREQ / 4U)
#define DELAY_600_MS       ((CPUCLK_FREQ / 1000U) * 600U)
#define CONTROL_LOOP_DELAY ((CPUCLK_FREQ / 1000U) * 15U)
#define GRAY8_SETTLE_CYCLES ((CPUCLK_FREQ / 1000000U) * 100U)

/* About 50 kHz with two delays in each software-I2C clock period. */
#define SOFT_I2C_DELAY_CYCLES  (CPUCLK_FREQ / 100000U)
#define SOFT_I2C_STRETCH_LIMIT (CPUCLK_FREQ / 1000U)

typedef enum {
    FOLLOW_STATE_WAITING_FOR_RST = 0,
    FOLLOW_STATE_ARMING,
    FOLLOW_STATE_RUNNING,
    FOLLOW_STATE_CORNER_APPROACH,
    FOLLOW_STATE_CORNER_PIVOT,
    FOLLOW_STATE_CORNER_SETTLE,
    FOLLOW_STATE_I2C_FAULT,
    FOLLOW_STATE_START_LINE_FAULT,
    FOLLOW_STATE_LINE_LOST,
    FOLLOW_STATE_WIDE_MARK_FAULT,
    FOLLOW_STATE_TURN_TIMEOUT,
    FOLLOW_STATE_IMU_FAULT
} FollowState;

typedef enum {
    CORNER_NONE = 0,
    CORNER_LEFT = -1,
    CORNER_RIGHT = 1
} CornerDirection;

#if GRAY8_CHANNEL0_IS_LEFT
#define LEFT_HALF_MASK   (0x0FU)
#define RIGHT_HALF_MASK  (0xF0U)
#define LEFT_OUTER_MASK  (0x03U)
#define RIGHT_OUTER_MASK (0xC0U)
#else
#define LEFT_HALF_MASK   (0xF0U)
#define RIGHT_HALF_MASK  (0x0FU)
#define LEFT_OUTER_MASK  (0xC0U)
#define RIGHT_OUTER_MASK (0x03U)
#endif
#define CENTER_SENSOR_MASK (0x18U)

volatile DL_SYSCTL_RESET_CAUSE gResetCause;
volatile bool gExternalResetDetected;
volatile FollowState gFollowState;
volatile CornerDirection gCornerDirection;
volatile uint8_t gGrayRaw;
volatile uint8_t gBlackMask;
volatile uint8_t gActiveSensorCount;
volatile bool gWhiteLevelIsHigh;
volatile int16_t gLinePosition;
volatile int16_t gFilteredError;
volatile int16_t gPidCorrection;
volatile int32_t gOfficialPatternIntegral;
volatile int16_t gLeftTarget;
volatile int16_t gRightTarget;
volatile int16_t gLeftCommand;
volatile int16_t gRightCommand;
volatile uint32_t gControlCycles;
volatile uint16_t gLostLineCycles;
volatile uint16_t gWideMarkCycles;
volatile uint16_t gCornerPhaseCycles;
volatile uint8_t gCornerConfirmCycles;
volatile uint8_t gCornerReacquireCycles;
volatile uint32_t gI2cErrorCount;
volatile float gCornerStartYaw;
volatile float gCornerSignedYawDelta;
volatile float gCornerTurnAngle;
volatile float gGyroTargetRateDps;
volatile float gGyroRateErrorDps;
volatile int16_t gGyroAssistCorrection;
volatile uint16_t gImuRawFailureCycles;
volatile uint16_t gImuYawStaleCycles;

static int16_t sPreviousError;
static int16_t sLastValidError;
static int16_t sFilteredDerivative;
static int32_t sIntegral;
static int8_t sOfficialPatternError;
static int8_t sOfficialPreviousError;
static int32_t sOfficialPatternIntegral;
static uint8_t sOfficialTurnFlag;
static CornerDirection sCornerCandidate;
static bool sCornerOldLineCleared;
static bool sCornerYawCaptured;
static float sFilteredGyroTargetRateDps;

static const int16_t sLineWeights[8] = {
    -350, -250, -150, -50, 50, 150, 250, 350
};

static void i2cDelay(void)
{
    delay_cycles(SOFT_I2C_DELAY_CYCLES);
}

static void driveLow(uint32_t pin)
{
    DL_GPIO_clearPins(GPIO_I2C_PORT, pin);
    DL_GPIO_enableOutput(GPIO_I2C_PORT, pin);
}

static void releaseLine(uint32_t pin)
{
    DL_GPIO_disableOutput(GPIO_I2C_PORT, pin);
}

static bool releaseClockAndWaitHigh(void)
{
    uint32_t timeout = SOFT_I2C_STRETCH_LIMIT;

    releaseLine(GPIO_I2C_SCL_PIN);
    while (DL_GPIO_readPins(GPIO_I2C_PORT, GPIO_I2C_SCL_PIN) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    return true;
}

static bool i2cStart(void)
{
    releaseLine(GPIO_I2C_SDA_PIN);
    if (!releaseClockAndWaitHigh()) {
        return false;
    }
    i2cDelay();

    if (DL_GPIO_readPins(GPIO_I2C_PORT, GPIO_I2C_SDA_PIN) == 0U) {
        return false;
    }

    driveLow(GPIO_I2C_SDA_PIN);
    i2cDelay();
    driveLow(GPIO_I2C_SCL_PIN);
    i2cDelay();
    return true;
}

static void i2cStop(void)
{
    driveLow(GPIO_I2C_SDA_PIN);
    i2cDelay();
    (void) releaseClockAndWaitHigh();
    i2cDelay();
    releaseLine(GPIO_I2C_SDA_PIN);
    i2cDelay();
}

static bool i2cWriteByte(uint8_t value)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            releaseLine(GPIO_I2C_SDA_PIN);
        } else {
            driveLow(GPIO_I2C_SDA_PIN);
        }
        i2cDelay();

        if (!releaseClockAndWaitHigh()) {
            driveLow(GPIO_I2C_SCL_PIN);
            return false;
        }
        i2cDelay();
        driveLow(GPIO_I2C_SCL_PIN);
        i2cDelay();
        value <<= 1U;
    }

    releaseLine(GPIO_I2C_SDA_PIN);
    i2cDelay();
    if (!releaseClockAndWaitHigh()) {
        driveLow(GPIO_I2C_SCL_PIN);
        return false;
    }

    i2cDelay();
    acknowledged =
        (DL_GPIO_readPins(GPIO_I2C_PORT, GPIO_I2C_SDA_PIN) == 0U);
    driveLow(GPIO_I2C_SCL_PIN);
    i2cDelay();
    return acknowledged;
}

static void i2cRecoverBus(void)
{
    uint8_t pulse;

    releaseLine(GPIO_I2C_SDA_PIN);
    for (pulse = 0U; pulse < 9U; pulse++) {
        driveLow(GPIO_I2C_SCL_PIN);
        i2cDelay();
        (void) releaseClockAndWaitHigh();
        i2cDelay();
    }
    i2cStop();
}

static bool i2cWriteRegister(
    uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t index;
    bool acknowledged;

    if (!i2cStart()) {
        return false;
    }

    acknowledged = i2cWriteByte((uint8_t) (address << 1U));
    if (acknowledged) {
        acknowledged = i2cWriteByte(reg);
    }

    for (index = 0U; (index < length) && acknowledged; index++) {
        acknowledged = i2cWriteByte(data[index]);
    }

    i2cStop();
    return acknowledged;
}

static bool setAllMotorSpeeds(
    int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    const int16_t motorSpeed[4] = {m1, m2, m3, m4};
    uint8_t payload[8];
    uint8_t motor;

    for (motor = 0U; motor < 4U; motor++) {
        uint16_t encoded = (uint16_t) motorSpeed[motor];
        payload[motor * 2U]      = (uint8_t) (encoded >> 8U);
        payload[motor * 2U + 1U] = (uint8_t) encoded;
    }

    return i2cWriteRegister(MOTOR_I2C_7BIT_ADDRESS,
        MOTOR_SPEED_REGISTER, payload, (uint8_t) sizeof(payload));
}

static int16_t clampTrackingWheelCommand(int32_t speed)
{
    /*
     * Four-wheel skid steering on cloth needs both sides to keep rolling.
     * Commands below this floor entered the observed low-speed/stall region.
     */
    if (speed < MIN_TRACKING_WHEEL_SPEED) {
        return MIN_TRACKING_WHEEL_SPEED;
    }
    if (speed > MAX_WHEEL_SPEED) {
        return MAX_WHEEL_SPEED;
    }
    return (int16_t) speed;
}

static bool setChassisSpeed(int16_t left, int16_t right)
{
    /*
     * User-verified chassis mapping:
     * M1 = left front, M2 = right front,
     * M3 = left rear,  M4 = right rear.
     *
     * M2 (right front) and M3 (left rear) need inverted signs for
     * physical forward travel. Therefore M1/M3 receive the left-side
     * command, while M2/M4 receive the right-side command.
     */
    return setAllMotorSpeeds(left, (int16_t) -right,
        (int16_t) -left, right);
}

static bool stopAllMotorsTwice(void)
{
    bool first = setAllMotorSpeeds(0, 0, 0, 0);
    delay_cycles(DELAY_100_MS);
    return setAllMotorSpeeds(0, 0, 0, 0) && first;
}

static void ledsOff(void)
{
    DL_GPIO_setPins(GPIO_LEDS_PORT, ALL_STATUS_LEDS);
}

static void greenLed(bool on)
{
    if (on) {
        DL_GPIO_clearPins(GPIO_LEDS_PORT, GREEN_LED_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LEDS_PORT, GREEN_LED_PIN);
    }
}

static void blueLed(bool on)
{
    if (on) {
        DL_GPIO_clearPins(GPIO_LEDS_PORT, BLUE_LED_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LEDS_PORT, BLUE_LED_PIN);
    }
}

static void buzzerOn(void)
{
    DL_TimerA_setCaptureCompareValue(
        BUZZER_INST, 30U, GPIO_BUZZER_C3_IDX);
}

static void buzzerOff(void)
{
    DL_TimerA_setCaptureCompareValue(
        BUZZER_INST, 0U, GPIO_BUZZER_C3_IDX);
}

static void beepShort(uint8_t count)
{
    uint8_t index;

    for (index = 0U; index < count; index++) {
        buzzerOn();
        delay_cycles(DELAY_60_MS);
        buzzerOff();
        delay_cycles(DELAY_90_MS);
    }
}

static void beepLong(uint8_t count)
{
    uint8_t index;

    for (index = 0U; index < count; index++) {
        buzzerOn();
        delay_cycles(DELAY_600_MS);
        buzzerOff();
        delay_cycles(DELAY_250_MS);
    }
}

static void gray8SelectChannel(uint8_t channel)
{
    uint32_t highPins = 0U;

    if ((channel & 0x01U) != 0U) {
        highPins |= GPIO_GRAY8_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U) {
        highPins |= GPIO_GRAY8_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U) {
        highPins |= GPIO_GRAY8_AD2_PIN;
    }

    DL_GPIO_clearPins(GPIO_GRAY8_PORT, GRAY8_ADDRESS_PINS);
    if (highPins != 0U) {
        DL_GPIO_setPins(GPIO_GRAY8_PORT, highPins);
    }
    delay_cycles(GRAY8_SETTLE_CYCLES);
}

static uint8_t gray8ReadRaw(void)
{
    uint8_t channel;
    uint8_t raw = 0U;

    for (channel = 0U; channel < 8U; channel++) {
        gray8SelectChannel(channel);
        if ((DL_GPIO_readPins(GPIO_GRAY8_PORT, GPIO_GRAY8_OUT_PIN) &
                GPIO_GRAY8_OUT_PIN) != 0U) {
            raw |= (uint8_t) (1U << channel);
        }
    }
    return raw;
}

/*
 * Three complete scans with a per-channel majority vote reject a single
 * reflection/noise spike without introducing a long moving-average delay.
 */
static uint8_t gray8ReadMajorityRaw(void)
{
    uint8_t first = gray8ReadRaw();
    uint8_t second = gray8ReadRaw();
    uint8_t third = gray8ReadRaw();

    return (uint8_t) ((first & second) | (first & third) | (second & third));
}

static uint8_t gray8CaptureStableRaw(void)
{
    uint8_t channel;
    uint8_t sample;
    uint8_t stableRaw = 0U;
    uint8_t highCount[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    for (sample = 0U; sample < STARTUP_SAMPLE_COUNT; sample++) {
        uint8_t raw = gray8ReadRaw();

        for (channel = 0U; channel < 8U; channel++) {
            if ((raw & (uint8_t) (1U << channel)) != 0U) {
                highCount[channel]++;
            }
        }
        delay_cycles(DELAY_5_MS);
    }

    for (channel = 0U; channel < 8U; channel++) {
        if (highCount[channel] >= (STARTUP_SAMPLE_COUNT / 2U)) {
            stableRaw |= (uint8_t) (1U << channel);
        }
    }
    return stableRaw;
}

static uint8_t countSetBits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count += (uint8_t) (value & 1U);
        value >>= 1U;
    }
    return count;
}

static uint8_t rawToBlackMask(uint8_t raw)
{
    if (gWhiteLevelIsHigh) {
        return (uint8_t) ~raw;
    }
    return raw;
}

static int16_t blackMaskToPosition(uint8_t blackMask, uint8_t activeCount)
{
    uint8_t channel;
    int32_t weightedSum = 0L;

    for (channel = 0U; channel < 8U; channel++) {
        if ((blackMask & (uint8_t) (1U << channel)) != 0U) {
#if GRAY8_CHANNEL0_IS_LEFT
            weightedSum += sLineWeights[channel];
#else
            weightedSum += sLineWeights[7U - channel];
#endif
        }
    }

    return (int16_t) (weightedSum / activeCount);
}

static int16_t clampPidCorrection(int32_t correction)
{
    if (correction > MAX_PID_CORRECTION) {
        return MAX_PID_CORRECTION;
    }
    if (correction < -MAX_PID_CORRECTION) {
        return (int16_t) -MAX_PID_CORRECTION;
    }
    return (int16_t) correction;
}

static void pidReset(int16_t error)
{
    sPreviousError = error;
    sFilteredDerivative = 0;
    sIntegral = 0L;
}

static int16_t pidUpdate(int16_t error)
{
    int32_t rawDerivative = (int32_t) error - sPreviousError;
    int32_t magnitude = error;
    int32_t output;

    /*
     * Filter derivative noise and only accumulate integral near center.
     * Ki remains zero initially, but this avoids windup if it is tuned later.
     */
    sFilteredDerivative =
        (int16_t) (((int32_t) sFilteredDerivative + rawDerivative) / 2L);

    if (magnitude < 0L) {
        magnitude = -magnitude;
    }
    if (magnitude <= 200L) {
        sIntegral += error;
    } else {
        sIntegral = (3L * sIntegral) / 4L;
    }

    if (sIntegral > PID_INTEGRAL_LIMIT) {
        sIntegral = PID_INTEGRAL_LIMIT;
    } else if (sIntegral < -PID_INTEGRAL_LIMIT) {
        sIntegral = -PID_INTEGRAL_LIMIT;
    }

    output = ((PID_KP_NUM * error) / PID_KP_DEN) +
             ((PID_KI_NUM * sIntegral) / PID_KI_DEN) +
             ((PID_KD_NUM * sFilteredDerivative) / PID_KD_DEN);
    sPreviousError = error;
    return clampPidCorrection(output);
}

static bool wasExternalNrst(DL_SYSCTL_RESET_CAUSE resetCause)
{
    return (resetCause == DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST) ||
           (resetCause == DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST);
}

static void stopAndLatchFault(FollowState fault, uint8_t beepCount, bool longBeeps)
{
    gFollowState = fault;
    gLeftCommand = 0;
    gRightCommand = 0;
    i2cRecoverBus();
    (void) stopAllMotorsTwice();
    greenLed(false);
    blueLed(true);

    if (longBeeps) {
        beepLong(beepCount);
    } else {
        beepShort(beepCount);
    }

    while (1) {
        __WFI();
    }
}

static CornerDirection detectCornerCandidate(uint8_t blackMask)
{
    uint8_t leftCount = countSetBits((uint8_t) (blackMask & LEFT_HALF_MASK));
    uint8_t rightCount =
        countSetBits((uint8_t) (blackMask & RIGHT_HALF_MASK));

    if ((leftCount >= 3U) && (rightCount <= 1U) &&
        ((blackMask & LEFT_OUTER_MASK) != 0U)) {
        return CORNER_LEFT;
    }
    if ((rightCount >= 3U) && (leftCount <= 1U) &&
        ((blackMask & RIGHT_OUTER_MASK) != 0U)) {
        return CORNER_RIGHT;
    }
    return CORNER_NONE;
}

static void commandChassisOrFault(int16_t left, int16_t right)
{
    gLeftCommand = left;
    gRightCommand = right;

    if (!setChassisSpeed(left, right)) {
        gI2cErrorCount++;
        stopAndLatchFault(FOLLOW_STATE_I2C_FAULT, 3U, true);
    }
}

static int16_t clampOfficialWheelCommand(int32_t speed)
{
    if (speed > OFFICIAL_MAX_WHEEL_SPEED) {
        return OFFICIAL_MAX_WHEEL_SPEED;
    }
    if (speed < -OFFICIAL_MAX_WHEEL_SPEED) {
        return (int16_t) -OFFICIAL_MAX_WHEEL_SPEED;
    }
    return (int16_t) speed;
}

/*
 * Adapter for the official deal_IRdata() API. The official I2C module reports
 * 0 for black and 1 for white. Our PA14/PA15-17 module is scanned locally and
 * stores 1 for black in gBlackMask, so the adapter reverses each selected bit.
 */
static void deal_IRdata(uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
    uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8)
{
    uint8_t physicalIndex;
    uint8_t channel;
    uint8_t whiteValue[8];

    for (physicalIndex = 0U; physicalIndex < 8U; physicalIndex++) {
#if GRAY8_CHANNEL0_IS_LEFT
        channel = physicalIndex;
#else
        channel = (uint8_t) (7U - physicalIndex);
#endif
        whiteValue[physicalIndex] =
            ((gBlackMask & (uint8_t) (1U << channel)) != 0U) ? 0U : 1U;
    }

    *x1 = whiteValue[0];
    *x2 = whiteValue[1];
    *x3 = whiteValue[2];
    *x4 = whiteValue[3];
    *x5 = whiteValue[4];
    *x6 = whiteValue[5];
    *x7 = whiteValue[6];
    *x8 = whiteValue[7];
}

/*
 * Official priority table. Unlisted patterns deliberately retain the previous
 * error, providing the same short-term direction memory as the reference.
 */
static int8_t officialPatternError(uint8_t x1, uint8_t x2, uint8_t x3,
    uint8_t x4, uint8_t x5, uint8_t x6, uint8_t x7, uint8_t x8)
{
    if ((x1 == 1U) && (x2 == 1U) && (x3 == 0U) && (x4 == 0U) &&
        (x5 == 0U) && (x6 == 0U) && (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = 15;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) && (x4 == 1U) &&
               (x5 == 1U) && (x6 == 1U) && (x7 == 1U) && (x8 == 1U)) {
        if (sOfficialTurnFlag == 0U) {
            sOfficialPatternError = 0;
            sOfficialTurnFlag = 1U;
        }
    } else if ((x1 == 0U) && (x2 == 0U) &&
               (x7 == 0U) && (x8 == 0U)) {
        sOfficialPatternError = 0;
        if (sOfficialTurnFlag == 1U) {
            sOfficialTurnFlag = 0U;
        }
    } else if ((x1 == 0U) && (x3 == 0U) && (x4 == 0U) &&
               (x5 == 0U) && (x8 == 0U)) {
        sOfficialPatternError = 0;
    } else if (((x1 == 0U) || (x2 == 0U)) && (x8 == 1U)) {
        sOfficialPatternError = -15;
    } else if (((x7 == 0U) || (x8 == 0U)) && (x1 == 1U)) {
        sOfficialPatternError = 15;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 0U) && (x5 == 1U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = -1;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 0U) &&
               (x4 == 0U) && (x5 == 1U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = -2;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 0U) &&
               (x4 == 1U) && (x5 == 1U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = -2;
    } else if ((x1 == 1U) && (x2 == 0U) && (x3 == 0U) &&
               (x4 == 1U) && (x5 == 1U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = -3;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 1U) && (x5 == 0U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = 1;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 1U) && (x5 == 0U) && (x6 == 0U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = 2;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 1U) && (x5 == 1U) && (x6 == 0U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = 2;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 1U) && (x5 == 1U) && (x6 == 0U) &&
               (x7 == 0U) && (x8 == 1U)) {
        sOfficialPatternError = 3;
    } else if ((x1 == 1U) && (x2 == 1U) && (x3 == 1U) &&
               (x4 == 0U) && (x5 == 0U) && (x6 == 1U) &&
               (x7 == 1U) && (x8 == 1U)) {
        sOfficialPatternError = 0;
    }

    return sOfficialPatternError;
}

static int16_t APP_ELE_PID_Calc(int8_t actualValue)
{
    int32_t magnitude = actualValue;
    int32_t output;

    /*
     * The gyro can make yaw rate reach zero while the optical error remains
     * persistently at -1 or +1. This bounded integral is intentionally
     * attached to the grayscale error, not to yaw rate: it keeps applying a
     * small lateral correction until the line returns to the center sensors.
     *
     * A sign reversal halves the stored correction, and a centered pattern
     * decays it slowly instead of dropping it abruptly. Sharp patterns bypass
     * the integral so a corner cannot inherit straight-line compensation.
     */
    if (magnitude < 0L) {
        magnitude = -magnitude;
    }
    if (magnitude > 3L) {
        sOfficialPatternIntegral = 0L;
    } else if (actualValue == 0) {
        sOfficialPatternIntegral =
            (7L * sOfficialPatternIntegral) / 8L;
    } else {
        if (((actualValue > 0) && (sOfficialPatternIntegral < 0L)) ||
            ((actualValue < 0) && (sOfficialPatternIntegral > 0L))) {
            sOfficialPatternIntegral /= 2L;
        }
        sOfficialPatternIntegral += actualValue;
    }

    if (sOfficialPatternIntegral > OFFICIAL_PATTERN_INTEGRAL_LIMIT) {
        sOfficialPatternIntegral = OFFICIAL_PATTERN_INTEGRAL_LIMIT;
    } else if (sOfficialPatternIntegral < -OFFICIAL_PATTERN_INTEGRAL_LIMIT) {
        sOfficialPatternIntegral = -OFFICIAL_PATTERN_INTEGRAL_LIMIT;
    }

    gOfficialPatternIntegral = sOfficialPatternIntegral;
    output =
        (OFFICIAL_TURN_KP * actualValue) +
        (OFFICIAL_TURN_KI * sOfficialPatternIntegral) +
        ((OFFICIAL_TURN_KD_NUM *
             ((int32_t) actualValue - sOfficialPreviousError)) /
            OFFICIAL_TURN_KD_DEN);
    sOfficialPreviousError = actualValue;
    return (int16_t) output;
}

static int16_t applyGyroRateFeedback(int16_t patternCorrection)
{
    float requestedTargetRate;
    float feedbackGain;
    int32_t gyroCorrection;

    /*
     * The official pattern table remains the primary steering controller.
     * Gyro feedback is deliberately limited to ordinary -3..+3 patterns so
     * it damps rapid yaw without fighting sharp/corner handling.
     */
    if (!gImuYawValid ||
        (sOfficialPatternError < -3) ||
        (sOfficialPatternError > 3) ||
        (gActiveSensorCount == 0U) ||
        (gActiveSensorCount >= 6U)) {
        sFilteredGyroTargetRateDps = 0.0f;
        gGyroTargetRateDps = 0.0f;
        gGyroRateErrorDps = 0.0f;
        gGyroAssistCorrection = 0;
        return patternCorrection;
    }

    requestedTargetRate =
        (float) sOfficialPatternError * GYRO_TARGET_RATE_PER_ERROR_DPS;
    sFilteredGyroTargetRateDps =
        (GYRO_TARGET_FILTER_NEW_WEIGHT * requestedTargetRate) +
        ((1.0f - GYRO_TARGET_FILTER_NEW_WEIGHT) *
            sFilteredGyroTargetRateDps);

    /*
     * Stronger damping is used only when the line is centered. While the line
     * is offset, a gentler gain lets the optical controller initiate the turn
     * without the gyro fighting it.
     */
    feedbackGain = (sOfficialPatternError == 0) ?
        GYRO_CENTER_DAMPING_GAIN : GYRO_RATE_FEEDBACK_GAIN;
    gGyroTargetRateDps = sFilteredGyroTargetRateDps;
    gGyroRateErrorDps = gImuGyroZDps - gGyroTargetRateDps;
    gyroCorrection =
        (int32_t) (gGyroRateErrorDps * feedbackGain);

    if (gyroCorrection > GYRO_RATE_MAX_CORRECTION) {
        gyroCorrection = GYRO_RATE_MAX_CORRECTION;
    } else if (gyroCorrection < -GYRO_RATE_MAX_CORRECTION) {
        gyroCorrection = -GYRO_RATE_MAX_CORRECTION;
    }

    gGyroAssistCorrection = (int16_t) gyroCorrection;
    return (int16_t) ((int32_t) patternCorrection + gyroCorrection);
}

/*
 * Hardware adapter for the official Motion_Car_Control(V_x, 0, V_z) API.
 * It preserves the verified local motor signs through setChassisSpeed().
 */
static void Motion_Car_Control(int16_t velocityX, int16_t velocityY,
    int16_t velocityZ)
{
    int32_t spinSpeed;

    (void) velocityY;
    spinSpeed = ((int32_t) velocityZ * OFFICIAL_CAR_APB) / 1000L;
    gLeftTarget =
        clampOfficialWheelCommand((int32_t) velocityX + spinSpeed);
    gRightTarget =
        clampOfficialWheelCommand((int32_t) velocityX - spinSpeed);
    commandChassisOrFault(gLeftTarget, gRightTarget);
}

static bool LineCheck(void)
{
    return (gActiveSensorCount != 0U);
}

static void LineWalking(void)
{
    uint8_t x1;
    uint8_t x2;
    uint8_t x3;
    uint8_t x4;
    uint8_t x5;
    uint8_t x6;
    uint8_t x7;
    uint8_t x8;

    deal_IRdata(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);
    sOfficialPatternError =
        officialPatternError(x1, x2, x3, x4, x5, x6, x7, x8);

    gLinePosition = (int16_t) (sOfficialPatternError * 100);
    gFilteredError = gLinePosition;
    gPidCorrection =
        applyGyroRateFeedback(APP_ELE_PID_Calc(sOfficialPatternError));

    if (LineCheck()) {
        sLastValidError = gLinePosition;
    }

    Motion_Car_Control(OFFICIAL_IRR_SPEED, 0, gPidCorrection);
}

static void beginCorner(CornerDirection direction)
{
    gCornerDirection = direction;
    gFollowState = FOLLOW_STATE_CORNER_APPROACH;
    gCornerPhaseCycles = 0U;
    gCornerReacquireCycles = 0U;
    gCornerSignedYawDelta = 0.0f;
    gCornerTurnAngle = 0.0f;
    sCornerOldLineCleared = false;
    sCornerYawCaptured = false;
    sCornerCandidate = CORNER_NONE;
    gCornerConfirmCycles = 0U;
    sOfficialPatternIntegral = 0L;
    gOfficialPatternIntegral = 0L;
    sFilteredGyroTargetRateDps = 0.0f;
    gGyroTargetRateDps = 0.0f;
    gGyroRateErrorDps = 0.0f;
    gGyroAssistCorrection = 0;
    pidReset(sLastValidError);
    greenLed(false);
    blueLed(true);
}

static void cornerControlStep(void)
{
    int16_t sensedPosition = 0;
    int16_t settleCorrection;
    int16_t pivotSpeed = CORNER_PIVOT_SPEED;
    bool lineShapeValid =
        (gActiveSensorCount >= 1U) && (gActiveSensorCount <= 3U);
    bool centerAligned = false;

    if (lineShapeValid) {
        sensedPosition =
            blackMaskToPosition(gBlackMask, gActiveSensorCount);
        centerAligned =
            ((gBlackMask & CENTER_SENSOR_MASK) != 0U) &&
            (sensedPosition >= -CORNER_CENTER_ERROR_LIMIT) &&
            (sensedPosition <= CORNER_CENTER_ERROR_LIMIT);
    }

    if (gFollowState == FOLLOW_STATE_CORNER_APPROACH) {
        commandChassisOrFault(CORNER_APPROACH_SPEED, CORNER_APPROACH_SPEED);
        gCornerPhaseCycles++;

        if (gCornerPhaseCycles >= CORNER_APPROACH_CYCLES) {
            if (!gImuYawValid) {
                stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 6U, false);
            }
            gCornerStartYaw = gImuYawUnwrappedDegrees;
            gCornerSignedYawDelta = 0.0f;
            gCornerTurnAngle = 0.0f;
            sCornerYawCaptured = true;
            gFollowState = FOLLOW_STATE_CORNER_PIVOT;
            gCornerPhaseCycles = 0U;
            gCornerReacquireCycles = 0U;
            sCornerOldLineCleared = false;
        }
        return;
    }

    if (gFollowState == FOLLOW_STATE_CORNER_PIVOT) {
        if (!sCornerYawCaptured || !gImuYawValid) {
            stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 6U, false);
        }

        gCornerSignedYawDelta =
            IMU_MPU6050_signedRelativeAngle(gCornerStartYaw);
        /*
         * Face-up installation: a left pivot must accumulate positive yaw,
         * and a right pivot must accumulate negative yaw. Converting the
         * expected direction to positive progress prevents a wrong-way pivot
         * from falsely satisfying the angle threshold.
         */
        if (gCornerDirection == CORNER_LEFT) {
            gCornerTurnAngle = gCornerSignedYawDelta;
        } else {
            gCornerTurnAngle = -gCornerSignedYawDelta;
        }
        if (gCornerTurnAngle >= CORNER_SLOW_ANGLE_DEGREES) {
            pivotSpeed = CORNER_PIVOT_SLOW_SPEED;
        }

        if (gCornerDirection == CORNER_LEFT) {
            commandChassisOrFault(
                (int16_t) -pivotSpeed, pivotSpeed);
        } else {
            commandChassisOrFault(
                pivotSpeed, (int16_t) -pivotSpeed);
        }

        gCornerPhaseCycles++;
        /*
         * Seeing the center briefly is not enough: the old corner edge must
         * disappear, a minimum pivot time must pass, and the new line must
         * remain close to center for several complete control loops.
         */
        if (!centerAligned) {
            sCornerOldLineCleared = true;
            gCornerReacquireCycles = 0U;
        } else if (sCornerOldLineCleared &&
                   (gCornerTurnAngle >=
                       CORNER_REACQUIRE_ANGLE_DEGREES)) {
            gCornerReacquireCycles++;
            if (gCornerReacquireCycles >= CORNER_REACQUIRE_CYCLES) {
                gFollowState = FOLLOW_STATE_CORNER_SETTLE;
                gCornerPhaseCycles = 0U;
                gLinePosition = sensedPosition;
                gFilteredError = gLinePosition;
                sLastValidError = gLinePosition;
                sOfficialPatternIntegral = 0L;
                gOfficialPatternIntegral = 0L;
                pidReset(gLinePosition);
            }
        }

        if (gCornerTurnAngle > CORNER_ABORT_ANGLE_DEGREES) {
            stopAndLatchFault(FOLLOW_STATE_TURN_TIMEOUT, 5U, false);
        }
        if (gCornerPhaseCycles > CORNER_TURN_TIMEOUT_CYCLES) {
            stopAndLatchFault(FOLLOW_STATE_TURN_TIMEOUT, 5U, false);
        }
        return;
    }

    /*
     * Use low-speed PID while leaving the corner instead of driving blindly
     * straight. If the line disappears, return to pivot/reacquisition.
     */
    if (!lineShapeValid) {
        gFollowState = FOLLOW_STATE_CORNER_PIVOT;
        gCornerPhaseCycles = 0U;
        gCornerReacquireCycles = 0U;
        sCornerOldLineCleared = true;
        if (gCornerDirection == CORNER_LEFT) {
            commandChassisOrFault(
                (int16_t) -CORNER_PIVOT_SPEED, CORNER_PIVOT_SPEED);
        } else {
            commandChassisOrFault(
                CORNER_PIVOT_SPEED, (int16_t) -CORNER_PIVOT_SPEED);
        }
        return;
    }

    gLinePosition = sensedPosition;
    gFilteredError = (int16_t) (
        ((int32_t) gFilteredError + (3L * gLinePosition)) / 4L);
    sLastValidError = gFilteredError;
    settleCorrection = pidUpdate(gFilteredError);
    if (settleCorrection > CORNER_SETTLE_MAX_CORRECTION) {
        settleCorrection = CORNER_SETTLE_MAX_CORRECTION;
    } else if (settleCorrection < -CORNER_SETTLE_MAX_CORRECTION) {
        settleCorrection = (int16_t) -CORNER_SETTLE_MAX_CORRECTION;
    }

    commandChassisOrFault(
        clampTrackingWheelCommand(
            (int32_t) CORNER_SETTLE_SPEED + settleCorrection),
        clampTrackingWheelCommand(
            (int32_t) CORNER_SETTLE_SPEED - settleCorrection));
    gCornerPhaseCycles++;
    if (gCornerPhaseCycles >= CORNER_SETTLE_CYCLES) {
        gFollowState = FOLLOW_STATE_RUNNING;
        gCornerDirection = CORNER_NONE;
        gCornerPhaseCycles = 0U;
        sCornerYawCaptured = false;
        sOfficialPatternIntegral = 0L;
        gOfficialPatternIntegral = 0L;
        sFilteredGyroTargetRateDps = 0.0f;
        gGyroTargetRateDps = 0.0f;
        gGyroRateErrorDps = 0.0f;
        gGyroAssistCorrection = 0;
        pidReset(sLastValidError);
        greenLed(true);
        blueLed(false);
    }
}

static void updateImuOrFault(void)
{
    if (IMU_MPU6050_update()) {
        gImuRawFailureCycles = 0U;
    } else {
        gImuRawFailureCycles++;
        if (gImuRawFailureCycles > IMU_RAW_FAILURE_LIMIT_CYCLES) {
            /* Six short beeps: raw MPU6050 register/I2C reads failed. */
            stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 6U, false);
        }
    }

    if (gImuYawFresh) {
        gImuYawStaleCycles = 0U;
    } else {
        gImuYawStaleCycles++;
        if (gImuYawStaleCycles > IMU_YAW_STALE_LIMIT_CYCLES) {
            /*
             * Seven short beeps: DMP produced no usable sample for too long.
             * Eight short beeps identifies repeated DMP yaw jumps separately.
             */
            if (gImuLastDmpSampleStatus == IMU_DMP_SAMPLE_YAW_JUMP) {
                stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 8U, false);
            } else {
                stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 7U, false);
            }
        }
    }
}

static void lineFollowerStep(void)
{
    CornerDirection cornerCandidate;

    updateImuOrFault();
    gGrayRaw = gray8ReadMajorityRaw();
    gBlackMask = rawToBlackMask(gGrayRaw);
    gActiveSensorCount = countSetBits(gBlackMask);

    if ((gFollowState == FOLLOW_STATE_CORNER_APPROACH) ||
        (gFollowState == FOLLOW_STATE_CORNER_PIVOT) ||
        (gFollowState == FOLLOW_STATE_CORNER_SETTLE)) {
        cornerControlStep();
        gControlCycles++;
        return;
    }

    cornerCandidate = detectCornerCandidate(gBlackMask);
    if (cornerCandidate != CORNER_NONE) {
        if (cornerCandidate == sCornerCandidate) {
            if (gCornerConfirmCycles < CORNER_CONFIRM_CYCLES) {
                gCornerConfirmCycles++;
            }
        } else {
            sCornerCandidate = cornerCandidate;
            gCornerConfirmCycles = 1U;
        }

        if (gCornerConfirmCycles >= CORNER_CONFIRM_CYCLES) {
            beginCorner(cornerCandidate);
            cornerControlStep();
            gControlCycles++;
            return;
        }
    } else {
        sCornerCandidate = CORNER_NONE;
        gCornerConfirmCycles = 0U;
    }

    if (gActiveSensorCount == 0U) {
        gLostLineCycles++;
        gWideMarkCycles = 0U;
        sIntegral = 0L;

        if (gLostLineCycles > LOST_LINE_LIMIT_CYCLES) {
            stopAndLatchFault(FOLLOW_STATE_LINE_LOST, 2U, true);
        }

        if (gLostLineCycles >= LOST_PIVOT_AFTER_CYCLES) {
            if (sLastValidError <= 0) {
                commandChassisOrFault(
                    (int16_t) -LOST_PIVOT_SPEED, LOST_PIVOT_SPEED);
            } else {
                commandChassisOrFault(
                    LOST_PIVOT_SPEED, (int16_t) -LOST_PIVOT_SPEED);
            }
            gControlCycles++;
            return;
        }
    } else if (gActiveSensorCount >= 6U) {
        gWideMarkCycles++;
        gLostLineCycles = 0U;
        sIntegral = 0L;

        if (gWideMarkCycles > WIDE_MARK_LIMIT_CYCLES) {
            stopAndLatchFault(FOLLOW_STATE_WIDE_MARK_FAULT, 3U, false);
        }
    } else {
        gLostLineCycles = 0U;
        gWideMarkCycles = 0U;
    }

    LineWalking();

    gControlCycles++;
}

int main(void)
{
    uint8_t initialStableRaw;
    uint8_t initialBlackMask;
    uint8_t initialBlackCount;
    uint8_t blink;

    /* Preserve the original cause before SysConfig initialization. */
    gResetCause = DL_SYSCTL_getResetCause();
    gExternalResetDetected = wasExternalNrst(gResetCause);

    SYSCFG_DL_init();
    ledsOff();
    buzzerOff();
    DL_Timer_startCounter(BUZZER_INST);

    releaseLine(GPIO_I2C_SDA_PIN);
    releaseLine(GPIO_I2C_SCL_PIN);
    delay_cycles(DELAY_100_MS);
    i2cRecoverBus();

    if (!stopAllMotorsTwice()) {
        stopAndLatchFault(FOLLOW_STATE_I2C_FAULT, 3U, true);
    }

    if (!gExternalResetDetected) {
        gFollowState = FOLLOW_STATE_WAITING_FOR_RST;
        greenLed(true);
        beepShort(1U);
        while (1) {
            __WFI();
        }
    }

    gFollowState = FOLLOW_STATE_ARMING;
    blueLed(true);

    /*
     * Keep the vehicle motionless while MotionApps self-test and local gyro-Z
     * zero-bias calibration run. Six short beeps report any IMU failure.
     */
    if (!IMU_MPU6050_init()) {
        stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 6U, false);
    }
    blueLed(false);

    /* Two-second placement interval: keep the car on a normal line section. */
    for (blink = 0U; blink < STARTUP_PLACEMENT_BLINKS; blink++) {
        greenLed(true);
        delay_cycles(DELAY_250_MS);
        greenLed(false);
        delay_cycles(DELAY_250_MS);
    }

    /*
     * With an 18 mm centered line, most sensors see white. Majority level
     * therefore discovers whether this module reports white as high or low.
     */
    initialStableRaw = gray8CaptureStableRaw();
    gWhiteLevelIsHigh = (countSetBits(initialStableRaw) >= 4U);
    initialBlackMask = rawToBlackMask(initialStableRaw);
    initialBlackCount = countSetBits(initialBlackMask);

    if ((initialBlackCount == 0U) || (initialBlackCount > 4U)) {
        stopAndLatchFault(FOLLOW_STATE_START_LINE_FAULT, 4U, false);
    }

    gGrayRaw = initialStableRaw;
    gBlackMask = initialBlackMask;
    gActiveSensorCount = initialBlackCount;
    gLinePosition = blackMaskToPosition(initialBlackMask, initialBlackCount);
    gFilteredError = gLinePosition;
    sLastValidError = gLinePosition;
    sCornerCandidate = CORNER_NONE;
    gCornerDirection = CORNER_NONE;
    sCornerYawCaptured = false;
    gCornerConfirmCycles = 0U;
    gCornerPhaseCycles = 0U;
    gCornerReacquireCycles = 0U;
    gLeftTarget = OFFICIAL_IRR_SPEED;
    gRightTarget = OFFICIAL_IRR_SPEED;
    sOfficialPatternError = 0;
    sOfficialPreviousError = 0;
    sOfficialPatternIntegral = 0L;
    gOfficialPatternIntegral = 0L;
    sOfficialTurnFlag = 0U;
    sFilteredGyroTargetRateDps = 0.0f;
    gGyroTargetRateDps = 0.0f;
    gGyroRateErrorDps = 0.0f;
    gGyroAssistCorrection = 0;
    pidReset(gLinePosition);

    /*
     * Calibration and placement delays can leave old DMP packets in FIFO.
     * Flush them immediately before motion so the first corner uses current
     * yaw rather than startup history.
     */
    if (!IMU_MPU6050_restartStream()) {
        stopAndLatchFault(FOLLOW_STATE_IMU_FAULT, 6U, false);
    }
    gImuRawFailureCycles = 0U;
    gImuYawStaleCycles = 0U;

    gFollowState = FOLLOW_STATE_RUNNING;
    greenLed(true);
    blueLed(false);
    beepShort(2U);

    while (1) {
        lineFollowerStep();
        delay_cycles(CONTROL_LOOP_DELAY);
    }
}
