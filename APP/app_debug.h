#ifndef APP_DEBUG_H_
#define APP_DEBUG_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint8_t line_raw;
    int16_t line_error;
    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;
    int32_t yaw_mdeg;
    int32_t yaw_rate_mdps;
    bool motor_ok;
    bool line_ok;
    bool imu_ok;
    uint8_t imu_status;
    uint8_t imu_who_am_i;
    bool oled_ok;
} AppDebugSnapshot;

bool AppDebug_Init(void);
void AppDebug_ShowBoot(const char *message);
void AppDebug_ShowState(uint8_t state, uint8_t fault);
void AppDebug_Refresh(const AppDebugSnapshot *snapshot);
bool AppDebug_IsAvailable(void);

#endif /* APP_DEBUG_H_ */
