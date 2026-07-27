#ifndef BSP_COMMON_SOFT_I2C_H_
#define BSP_COMMON_SOFT_I2C_H_

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t scl_pin;
    uint32_t sda_pin;
    uint32_t scl_iomux;
    uint32_t sda_iomux;
    uint32_t half_period_cycles;
    uint32_t stretch_timeout_cycles;
} SoftI2cBus;

void SoftI2c_Init(SoftI2cBus *bus);
bool SoftI2c_Probe(SoftI2cBus *bus, uint8_t address);
bool SoftI2c_WriteRegister(SoftI2cBus *bus, uint8_t address, uint8_t reg,
    const uint8_t *data, uint8_t length);
bool SoftI2c_ReadRegister(SoftI2cBus *bus, uint8_t address, uint8_t reg,
    uint8_t *data, uint8_t length);
void SoftI2c_Recover(SoftI2cBus *bus);

#endif /* BSP_COMMON_SOFT_I2C_H_ */
