#ifndef BSP_BUZZER_H_
#define BSP_BUZZER_H_

#include <stdbool.h>
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_PlayShort(uint8_t count);
void Buzzer_PlayLong(uint8_t count);
void Buzzer_Stop(void);
bool Buzzer_IsBusy(void);
void Buzzer_Tick1ms(void);

#endif /* BSP_BUZZER_H_ */
