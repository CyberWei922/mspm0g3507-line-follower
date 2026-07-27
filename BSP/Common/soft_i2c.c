#include "soft_i2c.h"

static void bus_delay(const SoftI2cBus *bus)
{
    delay_cycles(bus->half_period_cycles);
}

static void drive_low(const SoftI2cBus *bus, uint32_t pin)
{
    const uint32_t iomux = (pin == bus->scl_pin) ?
        bus->scl_iomux : bus->sda_iomux;

    /* 与之前验证过的 MPU6050 驱动一致：先切回数字输出，再拉低。 */
    DL_GPIO_initDigitalOutput(iomux);
    DL_GPIO_clearPins(bus->port, pin);
    DL_GPIO_enableOutput(bus->port, pin);
}

static void release_line(const SoftI2cBus *bus, uint32_t pin)
{
    const uint32_t iomux = (pin == bus->scl_pin) ?
        bus->scl_iomux : bus->sda_iomux;

    /* 释放I²C线时显式切成数字输入，避免输出模式残留影响从机应答。 */
    DL_GPIO_initDigitalInput(iomux);
    DL_GPIO_disableOutput(bus->port, pin);
}

static bool release_clock(const SoftI2cBus *bus)
{
    uint32_t timeout = bus->stretch_timeout_cycles;

    release_line(bus, bus->scl_pin);
    while ((DL_GPIO_readPins(bus->port, bus->scl_pin) & bus->scl_pin) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    return true;
}

static bool start_condition(const SoftI2cBus *bus)
{
    release_line(bus, bus->sda_pin);
    if (!release_clock(bus)) {
        return false;
    }
    bus_delay(bus);
    if ((DL_GPIO_readPins(bus->port, bus->sda_pin) & bus->sda_pin) == 0U) {
        return false;
    }
    drive_low(bus, bus->sda_pin);
    bus_delay(bus);
    drive_low(bus, bus->scl_pin);
    bus_delay(bus);
    return true;
}

static void stop_condition(const SoftI2cBus *bus)
{
    drive_low(bus, bus->sda_pin);
    bus_delay(bus);
    (void) release_clock(bus);
    bus_delay(bus);
    release_line(bus, bus->sda_pin);
    bus_delay(bus);
}

static bool write_byte(const SoftI2cBus *bus, uint8_t value)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; ++bit) {
        if ((value & 0x80U) != 0U) {
            release_line(bus, bus->sda_pin);
        } else {
            drive_low(bus, bus->sda_pin);
        }
        bus_delay(bus);
        if (!release_clock(bus)) {
            drive_low(bus, bus->scl_pin);
            return false;
        }
        bus_delay(bus);
        drive_low(bus, bus->scl_pin);
        value <<= 1U;
    }

    release_line(bus, bus->sda_pin);
    bus_delay(bus);
    if (!release_clock(bus)) {
        drive_low(bus, bus->scl_pin);
        return false;
    }
    acknowledged =
        ((DL_GPIO_readPins(bus->port, bus->sda_pin) & bus->sda_pin) == 0U);
    bus_delay(bus);
    drive_low(bus, bus->scl_pin);
    bus_delay(bus);
    return acknowledged;
}

static uint8_t read_byte(const SoftI2cBus *bus, bool acknowledge)
{
    uint8_t bit;
    uint8_t value = 0U;

    release_line(bus, bus->sda_pin);
    for (bit = 0U; bit < 8U; ++bit) {
        value <<= 1U;
        if (!release_clock(bus)) {
            drive_low(bus, bus->scl_pin);
            return value;
        }
        bus_delay(bus);
        if ((DL_GPIO_readPins(bus->port, bus->sda_pin) & bus->sda_pin) != 0U) {
            value |= 1U;
        }
        drive_low(bus, bus->scl_pin);
        bus_delay(bus);
    }

    if (acknowledge) {
        drive_low(bus, bus->sda_pin);
    } else {
        release_line(bus, bus->sda_pin);
    }
    bus_delay(bus);
    (void) release_clock(bus);
    bus_delay(bus);
    drive_low(bus, bus->scl_pin);
    release_line(bus, bus->sda_pin);
    bus_delay(bus);
    return value;
}

void SoftI2c_Init(SoftI2cBus *bus)
{
    release_line(bus, bus->scl_pin);
    release_line(bus, bus->sda_pin);
    SoftI2c_Recover(bus);
}

void SoftI2c_Recover(SoftI2cBus *bus)
{
    uint8_t pulse;

    release_line(bus, bus->sda_pin);
    for (pulse = 0U; pulse < 9U; ++pulse) {
        drive_low(bus, bus->scl_pin);
        bus_delay(bus);
        (void) release_clock(bus);
        bus_delay(bus);
    }
    stop_condition(bus);
}

bool SoftI2c_Probe(SoftI2cBus *bus, uint8_t address)
{
    bool ok;

    if (!start_condition(bus)) {
        SoftI2c_Recover(bus);
        return false;
    }
    ok = write_byte(bus, (uint8_t) (address << 1U));
    stop_condition(bus);
    return ok;
}

bool SoftI2c_WriteRegister(SoftI2cBus *bus, uint8_t address, uint8_t reg,
    const uint8_t *data, uint8_t length)
{
    uint8_t index;
    bool ok;

    if ((data == 0) && (length != 0U)) {
        return false;
    }
    if (!start_condition(bus)) {
        SoftI2c_Recover(bus);
        return false;
    }
    ok = write_byte(bus, (uint8_t) (address << 1U));
    if (ok) {
        ok = write_byte(bus, reg);
    }
    for (index = 0U; (index < length) && ok; ++index) {
        ok = write_byte(bus, data[index]);
    }
    stop_condition(bus);
    return ok;
}

bool SoftI2c_ReadRegister(SoftI2cBus *bus, uint8_t address, uint8_t reg,
    uint8_t *data, uint8_t length)
{
    uint8_t index;
    bool ok;

    if ((data == 0) || (length == 0U)) {
        return false;
    }
    if (!start_condition(bus)) {
        SoftI2c_Recover(bus);
        return false;
    }
    ok = write_byte(bus, (uint8_t) (address << 1U));
    if (ok) {
        ok = write_byte(bus, reg);
    }
    if (ok) {
        ok = start_condition(bus);
    }
    if (ok) {
        ok = write_byte(bus, (uint8_t) ((address << 1U) | 1U));
    }
    for (index = 0U; (index < length) && ok; ++index) {
        data[index] = read_byte(bus, index + 1U < length);
    }
    stop_condition(bus);
    return ok;
}
