#ifndef APP_STATE_MACHINE_H_
#define APP_STATE_MACHINE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_STATE_STOP = 0,
    APP_STATE_STRAIGHT_TRACKING = 1,
    APP_STATE_FREE_STRAIGHT = 2,
    APP_STATE_CURVE_TRACKING = 3,
    APP_STATE_LEFT_CORNER = 4,
    APP_STATE_RIGHT_CORNER = 5
} AppState;

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_SELF_TEST = 1,
    APP_FAULT_START_PATTERN = 2,
    APP_FAULT_LINE_LOST = 3,
    APP_FAULT_LINE_INVALID = 4,
    APP_FAULT_MOTOR_COMM = 5,
    APP_FAULT_IMU = 6,
    APP_FAULT_FREE_RUN_TIMEOUT = 7,
    APP_FAULT_LEFT_CORNER = 8,
    APP_FAULT_RIGHT_CORNER = 9,
    APP_FAULT_EMERGENCY_STOP = 10
} AppFault;

void AppStateMachine_Init(void);
void AppStateMachine_Run(void);

extern volatile AppState g_app_state;
extern volatile AppFault g_app_fault;
extern volatile bool g_self_test_motor_ok;
extern volatile bool g_self_test_line_ok;
extern volatile bool g_self_test_imu_ok;
extern volatile bool g_self_test_oled_ok;
extern volatile uint8_t g_line_raw;
extern volatile int16_t g_line_error;
extern volatile int16_t g_control_correction;
extern volatile int16_t g_left_speed;
extern volatile int16_t g_right_speed;
extern volatile int32_t g_yaw_mdeg;
extern volatile int32_t g_yaw_rate_mdps;

#endif /* APP_STATE_MACHINE_H_ */
