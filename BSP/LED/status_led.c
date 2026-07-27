#include "status_led.h"

#include "ti_msp_dl_config.h"

void StatusLed_Set(bool blue_on, bool green_on)
{
    if (blue_on) {
        DL_GPIO_setPins(STATUS_LEDS_PORT, STATUS_LEDS_BLUE_PIN);
    } else {
        DL_GPIO_clearPins(STATUS_LEDS_PORT, STATUS_LEDS_BLUE_PIN);
    }
    if (green_on) {
        DL_GPIO_setPins(STATUS_LEDS_PORT, STATUS_LEDS_GREEN_PIN);
    } else {
        DL_GPIO_clearPins(STATUS_LEDS_PORT, STATUS_LEDS_GREEN_PIN);
    }
}
