#include "system_time.h"

#include "BSP/Buzzer/buzzer.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_system_ms;

void SystemTime_Init(void)
{
    s_system_ms = 0U;
    NVIC_ClearPendingIRQ(SYSTEM_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(SYSTEM_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(SYSTEM_TIMER_INST);
}

uint32_t SystemTime_NowMs(void)
{
    return s_system_ms;
}

uint32_t SystemTime_ElapsedMs(uint32_t since_ms)
{
    return (uint32_t) (s_system_ms - since_ms);
}

void SystemTime_DelayMs(uint32_t delay_ms)
{
    const uint32_t start = SystemTime_NowMs();

    while (SystemTime_ElapsedMs(start) < delay_ms) {
        __WFI();
    }
}

void SYSTEM_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(SYSTEM_TIMER_INST) ==
            DL_TIMER_IIDX_ZERO) {
        ++s_system_ms;
        Buzzer_Tick1ms();
    }
}
