#include "app_turn_control.h"

#include "app_config.h"

#define CENTER_MASK      (0x18U)
#if LINE_CHANNEL0_IS_LEFT
#define LEFT_OUTER_BIT   (0x01U) /* bit0=X1，车体左侧 */
#define RIGHT_OUTER_BIT  (0x80U) /* bit7=X8，车体右侧 */
#define LEFT_INNER_MASK  (0x06U) /* X2/X3 */
#define RIGHT_INNER_MASK (0x60U) /* X6/X7 */
#define LEFT_SIDE_MASK   (0x07U)
#define RIGHT_SIDE_MASK  (0xE0U)
#else
#define LEFT_OUTER_BIT   (0x80U) /* bit7=X8，车体左侧 */
#define RIGHT_OUTER_BIT  (0x01U) /* bit0=X1，车体右侧 */
#define LEFT_INNER_MASK  (0x60U) /* X6/X7 */
#define RIGHT_INNER_MASK (0x06U) /* X2/X3 */
#define LEFT_SIDE_MASK   (0xE0U)
#define RIGHT_SIDE_MASK  (0x07U)
#endif

static TurnDirection s_candidate_direction;
static uint16_t s_candidate_ms;

static int32_t absolute_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t clamp_signed_speed(int32_t value)
{
    if (value > LINE_WHEEL_SPEED_MAX) {
        return LINE_WHEEL_SPEED_MAX;
    }
    if (value < -LINE_WHEEL_SPEED_MAX) {
        return (int16_t) -LINE_WHEEL_SPEED_MAX;
    }
    return (int16_t) value;
}

static void set_pivot_output(TurnDirection direction, int16_t speed,
    LineControlOutput *output)
{
    speed = clamp_signed_speed(speed);
    output->base_speed = 0;

    if (direction == TURN_DIRECTION_LEFT) {
        output->left_speed = (int16_t) -speed;
        output->right_speed = speed;
        output->correction = (int16_t) -speed;
    } else {
        output->left_speed = speed;
        output->right_speed = (int16_t) -speed;
        output->correction = speed;
    }
}

void AppTurnDetector_Reset(void)
{
    s_candidate_direction = TURN_DIRECTION_NONE;
    s_candidate_ms = 0U;
}

TurnDirection AppTurnDetector_Update(const LineObservation *line,
    uint16_t elapsed_ms, bool cooldown_active)
{
    const uint8_t black = line->black_mask;
    const bool center = (black & CENTER_MASK) != 0U;
    const bool x1 = (black & LEFT_OUTER_BIT) != 0U;
    const bool x8 = (black & RIGHT_OUTER_BIT) != 0U;
    const bool left_primary = center && x1 && !x8;
    const bool right_primary = center && x8 && !x1;
    const bool left_edge = !center && x1 &&
        ((black & LEFT_INNER_MASK) != 0U) && !x8;
    const bool right_edge = !center && x8 &&
        ((black & RIGHT_INNER_MASK) != 0U) && !x1;
    TurnDirection detected = TURN_DIRECTION_NONE;
    uint16_t required_ms = 0U;

    if (cooldown_active || (black == 0U) || (black == 0xFFU)) {
        AppTurnDetector_Reset();
        return TURN_DIRECTION_NONE;
    }
    if (left_primary) {
        detected = TURN_DIRECTION_LEFT;
        required_ms = CORNER_PRIMARY_CONFIRM_MS;
    } else if (right_primary) {
        detected = TURN_DIRECTION_RIGHT;
        required_ms = CORNER_PRIMARY_CONFIRM_MS;
    } else if (left_edge) {
        detected = TURN_DIRECTION_LEFT;
        required_ms = CORNER_EDGE_CONFIRM_MS;
    } else if (right_edge) {
        detected = TURN_DIRECTION_RIGHT;
        required_ms = CORNER_EDGE_CONFIRM_MS;
    }

    if (detected == TURN_DIRECTION_NONE) {
        AppTurnDetector_Reset();
        return TURN_DIRECTION_NONE;
    }
    if (detected != s_candidate_direction) {
        s_candidate_direction = detected;
        s_candidate_ms = elapsed_ms;
    } else if (s_candidate_ms < required_ms) {
        s_candidate_ms = (uint16_t) (s_candidate_ms + elapsed_ms);
    }
    if (s_candidate_ms < required_ms) {
        return TURN_DIRECTION_NONE;
    }

    AppTurnDetector_Reset();
    return detected;
}

void AppTurnControl_Begin(TurnController *controller,
    TurnDirection direction, uint32_t now_ms)
{
    controller->direction = direction;
    controller->phase = TURN_PHASE_LEAVE_OLD_LINE;
    controller->start_ms = now_ms;
    controller->reacquire_ms = 0U;
}

TurnResult AppTurnControl_Update(TurnController *controller,
    const LineObservation *line, int32_t relative_yaw_mdeg,
    uint32_t now_ms, LineControlOutput *output)
{
    const uint8_t black = line->black_mask;
    const bool center_seen = (black & CENTER_MASK) != 0U;
    const bool trigger_side_cleared =
        (controller->direction == TURN_DIRECTION_LEFT) ?
        ((black & LEFT_SIDE_MASK) == 0U) :
        ((black & RIGHT_SIDE_MASK) == 0U);
    const int32_t angle = absolute_i32(relative_yaw_mdeg);
    const uint32_t elapsed_ms = (uint32_t) (now_ms - controller->start_ms);

    if (angle >= CORNER_MAX_ANGLE_MDEG) {
        return TURN_RESULT_ANGLE_LIMIT;
    }
    if (elapsed_ms >= CORNER_TIMEOUT_MS) {
        return TURN_RESULT_TIMEOUT;
    }

    /* 先离开触发直角弯时的旧中心线，防止将旧线误判为出弯新线。 */
    if (controller->phase == TURN_PHASE_LEAVE_OLD_LINE) {
        if (!center_seen ||
            ((angle >= CORNER_LEAVE_MIN_ANGLE_MDEG) &&
                trigger_side_cleared)) {
            controller->phase = TURN_PHASE_REACQUIRE_NEW_LINE;
        }
    }

    /*
     * 与参考工程一致：不要求straight_line，也不要求55°。只排除全白、
     * 全黑，并在最短转动时间后检查中心探头和原触发侧是否已清空。
     */
    if ((controller->phase == TURN_PHASE_REACQUIRE_NEW_LINE) &&
        (black != 0U) && (black != 0xFFU) &&
        (elapsed_ms >= CORNER_MIN_TURN_MS) && center_seen &&
        trigger_side_cleared) {
        controller->reacquire_ms = (uint16_t) (
            controller->reacquire_ms + APP_CONTROL_PERIOD_MS);
        if (controller->reacquire_ms >= CORNER_REACQUIRE_CONFIRM_MS) {
            output->base_speed = 0;
            output->correction = 0;
            output->left_speed = 0;
            output->right_speed = 0;
            return TURN_RESULT_DONE;
        }
    } else {
        controller->reacquire_ms = 0U;
    }

    /* 参考工程固定125：左弯(-125,+125)，右弯(+125,-125)。 */
    set_pivot_output(controller->direction, CORNER_PIVOT_SPEED, output);
    return TURN_RESULT_ACTIVE;
}
