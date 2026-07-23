/*
 * Yahboom 8-channel black-line PID follower for MSPM0G3507.
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
 *   PB24 expansion-board buzzer (TIMA0 CCP3 PWM)
 */

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

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

#define BASE_SPEED          ((int16_t) 190)
#define MIN_CORNER_SPEED    ((int16_t) 110)
#define LOST_SEARCH_SPEED   ((int16_t) 100)
#define WIDE_MARK_SPEED     ((int16_t) 90)
#define MAX_WHEEL_SPEED     ((int16_t) 320)
#define MAX_PID_CORRECTION  ((int16_t) 160)
#define CORNER_APPROACH_SPEED ((int16_t) 110)
#define CORNER_PIVOT_SPEED    ((int16_t) 135)
#define CORNER_SETTLE_SPEED   ((int16_t) 105)
#define LOST_PIVOT_SPEED      ((int16_t) 90)

/* Fixed-point PID gains: Kp=0.45, Ki=0, Kd=0.70 initially. */
#define PID_KP_NUM          (45L)
#define PID_KP_DEN          (100L)
#define PID_KI_NUM          (0L)
#define PID_KI_DEN          (10000L)
#define PID_KD_NUM          (70L)
#define PID_KD_DEN          (100L)
#define PID_INTEGRAL_LIMIT  (8000L)

#define LOST_LINE_LIMIT_CYCLES (40U)
#define WIDE_MARK_LIMIT_CYCLES (25U)
#define STARTUP_SAMPLE_COUNT   (32U)
#define CORNER_CONFIRM_CYCLES       (2U)
#define CORNER_APPROACH_CYCLES      (8U)
#define CORNER_REACQUIRE_CYCLES     (2U)
#define CORNER_SETTLE_CYCLES        (6U)
#define CORNER_TURN_TIMEOUT_CYCLES  (60U)
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
    FOLLOW_STATE_TURN_TIMEOUT
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
volatile int16_t gLeftCommand;
volatile int16_t gRightCommand;
volatile uint32_t gControlCycles;
volatile uint16_t gLostLineCycles;
volatile uint16_t gWideMarkCycles;
volatile uint16_t gCornerPhaseCycles;
volatile uint8_t gCornerConfirmCycles;
volatile uint8_t gCornerReacquireCycles;
volatile uint32_t gI2cErrorCount;

static int16_t sPreviousError;
static int16_t sLastValidError;
static int16_t sFilteredDerivative;
static int32_t sIntegral;
static CornerDirection sCornerCandidate;
static bool sCornerOldLineCleared;

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

static int16_t clampWheelCommand(int32_t speed)
{
    if (speed < 0L) {
        return 0;
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
        (int16_t) (((3L * sFilteredDerivative) + rawDerivative) / 4L);

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

static int16_t scheduledBaseSpeed(int16_t error)
{
    int32_t magnitude = error;
    int32_t reduction;

    if (magnitude < 0L) {
        magnitude = -magnitude;
    }
    if (magnitude > 350L) {
        magnitude = 350L;
    }

    reduction = ((BASE_SPEED - MIN_CORNER_SPEED) * magnitude) / 350L;
    return (int16_t) (BASE_SPEED - reduction);
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

static void beginCorner(CornerDirection direction)
{
    gCornerDirection = direction;
    gFollowState = FOLLOW_STATE_CORNER_APPROACH;
    gCornerPhaseCycles = 0U;
    gCornerReacquireCycles = 0U;
    sCornerOldLineCleared = false;
    sCornerCandidate = CORNER_NONE;
    gCornerConfirmCycles = 0U;
    pidReset(sLastValidError);
    greenLed(false);
    blueLed(true);
}

static void cornerControlStep(void)
{
    bool centerDetected =
        ((gBlackMask & CENTER_SENSOR_MASK) != 0U) &&
        (gActiveSensorCount >= 1U) && (gActiveSensorCount <= 4U);

    if (gFollowState == FOLLOW_STATE_CORNER_APPROACH) {
        commandChassisOrFault(CORNER_APPROACH_SPEED, CORNER_APPROACH_SPEED);
        gCornerPhaseCycles++;

        if (gCornerPhaseCycles >= CORNER_APPROACH_CYCLES) {
            gFollowState = FOLLOW_STATE_CORNER_PIVOT;
            gCornerPhaseCycles = 0U;
            gCornerReacquireCycles = 0U;
            sCornerOldLineCleared = false;
        }
        return;
    }

    if (gFollowState == FOLLOW_STATE_CORNER_PIVOT) {
        if (gCornerDirection == CORNER_LEFT) {
            commandChassisOrFault(
                (int16_t) -CORNER_PIVOT_SPEED, CORNER_PIVOT_SPEED);
        } else {
            commandChassisOrFault(
                CORNER_PIVOT_SPEED, (int16_t) -CORNER_PIVOT_SPEED);
        }

        gCornerPhaseCycles++;
        if (!centerDetected) {
            sCornerOldLineCleared = true;
            gCornerReacquireCycles = 0U;
        } else if (sCornerOldLineCleared) {
            gCornerReacquireCycles++;
            if (gCornerReacquireCycles >= CORNER_REACQUIRE_CYCLES) {
                gFollowState = FOLLOW_STATE_CORNER_SETTLE;
                gCornerPhaseCycles = 0U;
                gLinePosition =
                    blackMaskToPosition(gBlackMask, gActiveSensorCount);
                gFilteredError = gLinePosition;
                sLastValidError = gLinePosition;
                pidReset(gLinePosition);
            }
        }

        if (gCornerPhaseCycles > CORNER_TURN_TIMEOUT_CYCLES) {
            stopAndLatchFault(FOLLOW_STATE_TURN_TIMEOUT, 5U, false);
        }
        return;
    }

    /* Drive slowly across the corner before returning control to PID. */
    commandChassisOrFault(CORNER_SETTLE_SPEED, CORNER_SETTLE_SPEED);
    gCornerPhaseCycles++;
    if (gCornerPhaseCycles >= CORNER_SETTLE_CYCLES) {
        gFollowState = FOLLOW_STATE_RUNNING;
        gCornerDirection = CORNER_NONE;
        gCornerPhaseCycles = 0U;
        pidReset(sLastValidError);
        greenLed(true);
        blueLed(false);
    }
}

static void lineFollowerStep(void)
{
    CornerDirection cornerCandidate;
    int16_t controlError;
    int16_t baseSpeed;

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

        controlError = sLastValidError;
        baseSpeed = LOST_SEARCH_SPEED;
    } else if (gActiveSensorCount >= 6U) {
        gWideMarkCycles++;
        gLostLineCycles = 0U;
        sIntegral = 0L;

        if (gWideMarkCycles > WIDE_MARK_LIMIT_CYCLES) {
            stopAndLatchFault(FOLLOW_STATE_WIDE_MARK_FAULT, 3U, false);
        }

        controlError = sLastValidError;
        baseSpeed = WIDE_MARK_SPEED;
    } else {
        gLostLineCycles = 0U;
        gWideMarkCycles = 0U;
        gLinePosition =
            blackMaskToPosition(gBlackMask, gActiveSensorCount);

        /* One-pole digital smoothing: 50% previous, 50% newest. */
        gFilteredError =
            (int16_t) (((int32_t) gFilteredError + gLinePosition) / 2L);
        controlError = gFilteredError;
        sLastValidError = controlError;
        baseSpeed = scheduledBaseSpeed(controlError);
    }

    gPidCorrection = pidUpdate(controlError);
    gLeftCommand =
        clampWheelCommand((int32_t) baseSpeed + gPidCorrection);
    gRightCommand =
        clampWheelCommand((int32_t) baseSpeed - gPidCorrection);

    commandChassisOrFault(gLeftCommand, gRightCommand);

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
    blueLed(false);

    /* Two-second placement interval: keep the car on a normal line section. */
    for (blink = 0U; blink < 4U; blink++) {
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
    gCornerConfirmCycles = 0U;
    gCornerPhaseCycles = 0U;
    gCornerReacquireCycles = 0U;
    pidReset(gLinePosition);

    gFollowState = FOLLOW_STATE_RUNNING;
    greenLed(true);
    blueLed(false);
    beepShort(2U);

    while (1) {
        lineFollowerStep();
        delay_cycles(CONTROL_LOOP_DELAY);
    }
}
