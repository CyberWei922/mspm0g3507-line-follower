#include "key.h"

#include <stdint.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    bool sampled;
    bool stable;
    bool pressed_event;
    uint8_t stable_ms;
} KeyFilter;

static KeyFilter s_keys[KEY_COUNT];

static bool read_pressed(KeyId key)
{
    switch (key) {
        case KEY_K1:
            return (DL_GPIO_readPins(BUTTONS_A_PORT, BUTTONS_A_K1_PIN) &
                BUTTONS_A_K1_PIN) == 0U;
        case KEY_K2:
            return (DL_GPIO_readPins(BUTTONS_B_PORT, BUTTONS_B_K2_PIN) &
                BUTTONS_B_K2_PIN) == 0U;
        case KEY_K3:
            return (DL_GPIO_readPins(BUTTONS_B_PORT, BUTTONS_B_K3_PIN) &
                BUTTONS_B_K3_PIN) == 0U;
        case KEY_K4:
            return (DL_GPIO_readPins(BUTTONS_A_PORT, BUTTONS_A_K4_PIN) &
                BUTTONS_A_K4_PIN) == 0U;
        default:
            return false;
    }
}

void Key_Init(void)
{
    uint8_t key;

    for (key = 0U; key < KEY_COUNT; ++key) {
        s_keys[key].sampled = read_pressed((KeyId) key);
        s_keys[key].stable = s_keys[key].sampled;
        s_keys[key].pressed_event = false;
        s_keys[key].stable_ms = 0U;
    }
}

void Key_Update(void)
{
    uint8_t key;

    for (key = 0U; key < KEY_COUNT; ++key) {
        KeyFilter *filter = &s_keys[key];
        const bool pressed = read_pressed((KeyId) key);

        if (pressed != filter->sampled) {
            filter->sampled = pressed;
            filter->stable_ms = 0U;
        } else if (filter->stable_ms < KEY_DEBOUNCE_MS) {
            filter->stable_ms = (uint8_t) (filter->stable_ms +
                APP_KEY_PERIOD_MS);
        }

        if ((filter->stable_ms >= KEY_DEBOUNCE_MS) &&
                (filter->stable != filter->sampled)) {
            filter->stable = filter->sampled;
            if (filter->stable) {
                filter->pressed_event = true;
            }
        }
    }
}

bool Key_TakePressed(KeyId key)
{
    bool event;

    if (key >= KEY_COUNT) {
        return false;
    }
    event = s_keys[key].pressed_event;
    s_keys[key].pressed_event = false;
    return event;
}

bool Key_IsPressed(KeyId key)
{
    return (key < KEY_COUNT) ? s_keys[key].stable : false;
}
