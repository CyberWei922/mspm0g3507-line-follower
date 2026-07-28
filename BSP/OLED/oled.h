#ifndef BSP_OLED_H_
#define BSP_OLED_H_

#include <stdbool.h>
#include <stdint.h>

bool Oled_Init(void);
void Oled_Clear(void);
void Oled_ClearPage(uint8_t page);
void Oled_ShowString(uint8_t x, uint8_t page, const char *text);
bool Oled_RefreshPages(uint8_t first_page, uint8_t page_count);
bool Oled_Refresh(void);
uint8_t Oled_GetAddress(void);

#endif /* BSP_OLED_H_ */
