#ifndef BSP_MOTOR_H_
#define BSP_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

void Motor_Init(void);
bool Motor_Probe(void);
bool Motor_SetFourWheel(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
bool Motor_SetChassisSpeed(int16_t left_speed, int16_t right_speed);
bool Motor_Stop(void);

extern volatile int16_t g_motor_left_command;
extern volatile int16_t g_motor_right_command;
extern volatile uint32_t g_motor_error_count;

#endif /* BSP_MOTOR_H_ */
