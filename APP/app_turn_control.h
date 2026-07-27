#ifndef APP_TURN_CONTROL_H_
#define APP_TURN_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "APP/app_line_control.h"
#include "BSP/Eight_Tracking/eight_tracking.h"

typedef enum {
    TURN_DIRECTION_NONE = 0,
    TURN_DIRECTION_LEFT = -1,
    TURN_DIRECTION_RIGHT = 1
} TurnDirection;

typedef enum {
    TURN_PHASE_IDLE = 0,
    TURN_PHASE_LEAVE_OLD_LINE,
    TURN_PHASE_REACQUIRE_NEW_LINE
} TurnPhase;

typedef enum {
    TURN_RESULT_ACTIVE = 0,
    TURN_RESULT_DONE,
    TURN_RESULT_LINE_LOST,
    TURN_RESULT_TIMEOUT,
    TURN_RESULT_ANGLE_LIMIT
} TurnResult;

typedef struct {
    TurnDirection direction;
    TurnPhase phase;
    uint32_t start_ms;
    uint16_t reacquire_ms;
} TurnController;

void AppTurnDetector_Reset(void);
TurnDirection AppTurnDetector_Update(const LineObservation *line,
    uint16_t elapsed_ms, bool cooldown_active);
void AppTurnControl_Begin(TurnController *controller,
    TurnDirection direction, uint32_t now_ms);
TurnResult AppTurnControl_Update(TurnController *controller,
    const LineObservation *line, int32_t relative_yaw_mdeg,
    uint32_t now_ms, LineControlOutput *output);

#endif /* APP_TURN_CONTROL_H_ */
