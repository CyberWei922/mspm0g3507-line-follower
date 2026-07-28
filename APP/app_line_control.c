#include "app_line_control.h"

#include "app_config.h"

static int16_t s_previous_error;

static int16_t clamp_i16(int32_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return (int16_t) value;
}

static int16_t choose_base_speed(const LineObservation *line,
    LineControlProfile profile)
{
    const uint8_t black = line->black_mask;

    if ((black & 0x81U) != 0U) {      /* X1或X8 */
        return LINE_SPEED_OUTER;
    }
    if ((black & 0x42U) != 0U) {      /* X2或X7 */
        return LINE_SPEED_EDGE;
    }
    if ((black & 0x24U) != 0U) {      /* X3或X6 */
        return LINE_SPEED_CURVE;
    }
    return (profile == LINE_CONTROL_CURVE) ?
        LINE_CURVE_CENTER_SPEED : LINE_SPEED_STRAIGHT;
}

void AppLineControl_Reset(int16_t current_error)
{
    s_previous_error = current_error;
}

void AppLineControl_Update(const LineObservation *line,
    LineControlProfile profile, int32_t gyro_rate_mdps,
    LineControlOutput *output)
{
    int32_t correction;
    int16_t kp;
    int16_t kd;
    int16_t max_correction;
    int16_t gyro_denominator;
    int16_t error_magnitude;

    if ((line == 0) || (output == 0)) {
        return;
    }

    if (profile == LINE_CONTROL_CURVE) {
        kp = LINE_CURVE_KP;
        kd = LINE_CURVE_KD;
        max_correction = LINE_CURVE_MAX_CORRECTION;
        gyro_denominator = LINE_CURVE_GYRO_DAMP_DEN;
    } else {
        kp = LINE_STRAIGHT_KP;
        kd = LINE_STRAIGHT_KD;
        max_correction = LINE_STRAIGHT_MAX_CORRECTION;
        gyro_denominator = LINE_STRAIGHT_GYRO_DAMP_DEN;
    }

    /*
     * 真正的PD：D项使用本周期误差减上周期误差。
     * 陀螺仪只提供当前角速度阻尼，不用累计Yaw强行拉直车辆。
     */
    correction = (int32_t) kp * line->error +
        (int32_t) kd * (line->error - s_previous_error) +
        gyro_rate_mdps / (1000L * gyro_denominator);

    /*
     * 状态1下，误差绝对值2对应典型X3+X4/X5+X6，绝对值3还包括
     * 只压到X3或X6的情况。两者都属于刚离开中心、尚未达到状态3门槛，
     * 立即追加同向修正；回到中心后D项反向制动，减少越过中心的概率。
     */
    error_magnitude = (line->error < 0) ?
        (int16_t) -line->error : line->error;
    if ((profile == LINE_CONTROL_STRAIGHT) &&
        (error_magnitude >= 2) &&
        (error_magnitude < CURVE_ENTER_ERROR_MIN)) {
        if (line->error < 0) {
            correction -= LINE_STRAIGHT_NEAR_CENTER_BOOST;
        } else {
            correction += LINE_STRAIGHT_NEAR_CENTER_BOOST;
        }
    }
    correction = clamp_i16(correction,
        (int16_t) -max_correction, max_correction);

    output->base_speed = choose_base_speed(line, profile);
    output->correction = (int16_t) correction;
    output->left_speed = clamp_i16(
        (int32_t) output->base_speed + correction,
        0, LINE_WHEEL_SPEED_MAX);
    output->right_speed = clamp_i16(
        (int32_t) output->base_speed - correction,
        0, LINE_WHEEL_SPEED_MAX);
    s_previous_error = line->error;
}

void AppLineControl_HeadingStraight(int32_t heading_error_mdeg,
    LineControlOutput *output)
{
    int32_t correction;

    if (output == 0) {
        return;
    }
    correction = (heading_error_mdeg * FREE_RUN_HEADING_KP_NUM) /
        FREE_RUN_HEADING_KP_DEN;
    correction = clamp_i16(correction,
        (int16_t) -FREE_RUN_MAX_CORRECTION, FREE_RUN_MAX_CORRECTION);

    output->base_speed = FREE_RUN_SPEED;
    output->correction = (int16_t) correction;
    output->left_speed = clamp_i16(FREE_RUN_SPEED + correction,
        0, LINE_WHEEL_SPEED_MAX);
    output->right_speed = clamp_i16(FREE_RUN_SPEED - correction,
        0, LINE_WHEEL_SPEED_MAX);
}
