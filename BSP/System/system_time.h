#ifndef BSP_SYSTEM_TIME_H_
#define BSP_SYSTEM_TIME_H_

#include <stdint.h>

void SystemTime_Init(void);
uint32_t SystemTime_NowMs(void);
uint32_t SystemTime_ElapsedMs(uint32_t since_ms);
void SystemTime_DelayMs(uint32_t delay_ms);

#endif /* BSP_SYSTEM_TIME_H_ */
