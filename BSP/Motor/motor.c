#include "motor.h"

#include "BSP/Common/soft_i2c.h"
#include "ti_msp_dl_config.h"

#define MOTOR_ADDRESS        (0x26U)
#define MOTOR_SPEED_REGISTER (0x06U)
#define MOTOR_COMMAND_LIMIT  (1000)

volatile int16_t g_motor_left_command;
volatile int16_t g_motor_right_command;
volatile uint32_t g_motor_error_count;

static SoftI2cBus s_motor_bus;

static int16_t clamp_command(int32_t value)
{
    if (value > MOTOR_COMMAND_LIMIT) {
        return MOTOR_COMMAND_LIMIT;
    }
    if (value < -MOTOR_COMMAND_LIMIT) {
        return -MOTOR_COMMAND_LIMIT;
    }
    return (int16_t) value;
}

void Motor_Init(void)
{
    s_motor_bus.port = MOTOR_I2C_PORT;
    s_motor_bus.scl_pin = MOTOR_I2C_MOTOR_SCL_PIN;
    s_motor_bus.sda_pin = MOTOR_I2C_MOTOR_SDA_PIN;
    s_motor_bus.scl_iomux = MOTOR_I2C_MOTOR_SCL_IOMUX;
    s_motor_bus.sda_iomux = MOTOR_I2C_MOTOR_SDA_IOMUX;
    s_motor_bus.half_period_cycles = CPUCLK_FREQ / 100000U;
    s_motor_bus.stretch_timeout_cycles = CPUCLK_FREQ / 1000U;
    g_motor_left_command = 0;
    g_motor_right_command = 0;
    g_motor_error_count = 0U;
    SoftI2c_Init(&s_motor_bus);
}

bool Motor_Probe(void)
{
    return SoftI2c_Probe(&s_motor_bus, MOTOR_ADDRESS);
}

bool Motor_SetFourWheel(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    const int16_t speed[4] = {
        clamp_command(m1), clamp_command(m2),
        clamp_command(m3), clamp_command(m4)
    };
    uint8_t payload[8];
    uint8_t index;
    bool ok;

    for (index = 0U; index < 4U; ++index) {
        const uint16_t encoded = (uint16_t) speed[index];
        payload[index * 2U] = (uint8_t) (encoded >> 8U);
        payload[index * 2U + 1U] = (uint8_t) encoded;
    }
    ok = SoftI2c_WriteRegister(&s_motor_bus, MOTOR_ADDRESS,
        MOTOR_SPEED_REGISTER, payload, sizeof(payload));
    if (!ok) {
        ++g_motor_error_count;
        SoftI2c_Recover(&s_motor_bus);
    }
    return ok;
}

bool Motor_SetChassisSpeed(int16_t left_speed, int16_t right_speed)
{
    bool ok;

    left_speed = clamp_command(left_speed);
    right_speed = clamp_command(right_speed);
    g_motor_left_command = left_speed;
    g_motor_right_command = right_speed;

    /*
     * 已实车确认：M1左前、M2右前、M3左后、M4右后；
     * M2和M3机械方向相反，所以发送时翻转符号。
     */
    ok = Motor_SetFourWheel(left_speed, (int16_t) -right_speed,
        (int16_t) -left_speed, right_speed);
    return ok;
}

bool Motor_Stop(void)
{
    g_motor_left_command = 0;
    g_motor_right_command = 0;
    return Motor_SetFourWheel(0, 0, 0, 0);
}
