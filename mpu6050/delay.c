#include "delay.h"

#include "ti_msp_dl_config.h"

void delay_us(unsigned long microseconds)
{
    const uint32_t cyclesPerMicrosecond = CPUCLK_FREQ / 1000000U;

    while (microseconds-- != 0UL) {
        delay_cycles(cyclesPerMicrosecond);
    }
}

void delay_ms(unsigned long milliseconds)
{
    while (milliseconds-- != 0UL) {
        delay_us(1000UL);
    }
}
