#ifndef BSP_KEY_H_
#define BSP_KEY_H_

#include <stdbool.h>

typedef enum {
    KEY_K1 = 0,
    KEY_K2,
    KEY_K3,
    KEY_K4,
    KEY_COUNT
} KeyId;

void Key_Init(void);
void Key_Update(void);
bool Key_TakePressed(KeyId key);
bool Key_IsPressed(KeyId key);

#endif /* BSP_KEY_H_ */
