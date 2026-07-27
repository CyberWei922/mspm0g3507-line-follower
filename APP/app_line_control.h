#ifndef APP_LINE_CONTROL_H_
#define APP_LINE_CONTROL_H_

#include <stdint.h>

#include "BSP/Eight_Tracking/eight_tracking.h"

typedef enum {
    LINE_CONTROL_STRAIGHT = 0,
    LINE_CONTROL_CURVE
} LineControlProfile;

typedef struct {
    int16_t base_speed;
    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;
} LineControlOutput;

void AppLineControl_Reset(int16_t current_error);
void AppLineControl_Update(const LineObservation *line,
    LineControlProfile profile, int32_t gyro_rate_mdps,
    LineControlOutput *output);
void AppLineControl_HeadingStraight(int32_t heading_error_mdeg,
    LineControlOutput *output);

#endif /* APP_LINE_CONTROL_H_ */
