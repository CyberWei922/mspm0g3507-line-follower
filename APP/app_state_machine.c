#include "app_state_machine.h"

#include "app_config.h"
#include "APP/app_debug.h"
#include "APP/app_line_control.h"
#include "APP/app_turn_control.h"
#include "BSP/Buzzer/buzzer.h"
#include "BSP/Eight_Tracking/eight_tracking.h"
#include "BSP/Key/key.h"
#include "BSP/LED/status_led.h"
#include "BSP/Motor/motor.h"
#include "BSP/MPU6050/mpu6050.h"
#include "BSP/System/system_time.h"

volatile AppState g_app_state = APP_STATE_STOP;
volatile AppFault g_app_fault = APP_FAULT_NONE;
volatile bool g_self_test_motor_ok;
volatile bool g_self_test_line_ok;
volatile bool g_self_test_imu_ok;
volatile bool g_self_test_oled_ok;
volatile uint8_t g_line_raw;
volatile int16_t g_line_error;
volatile int16_t g_control_correction;
volatile int16_t g_left_speed;
volatile int16_t g_right_speed;
volatile int32_t g_yaw_mdeg;
volatile int32_t g_yaw_rate_mdps;

static LineObservation s_line;
static LineControlOutput s_output;
static TurnController s_turn;
static uint32_t s_last_key_ms;
static uint32_t s_last_control_ms;
static uint32_t s_state_entry_ms;
static int32_t s_free_heading_mdeg;
static uint16_t s_line_loss_ms;
static uint16_t s_invalid_ms;
static uint16_t s_curve_enter_ms;
static uint16_t s_curve_exit_ms;
static uint16_t s_free_line_ms;
static uint16_t s_corner_cooldown_ms;
static int8_t s_curve_candidate_side;
static bool s_oled_event_pending;
static bool s_oled_full_refresh_pending;

static void refresh_debug(void);

static void queue_oled_event(bool full_refresh)
{
    s_oled_event_pending = true;
    s_oled_full_refresh_pending = full_refresh;
}

static void service_oled_event(void)
{
    if (!s_oled_event_pending) {
        return;
    }
    s_oled_event_pending = false;
    if (s_oled_full_refresh_pending) {
        refresh_debug();
    } else {
        AppDebug_ShowState((uint8_t) g_app_state, (uint8_t) g_app_fault);
    }
    s_oled_full_refresh_pending = false;
}

static bool init_oled_with_retry(void)
{
    uint8_t attempt;

#if APP_ENABLE_OLED
    for (attempt = 0U; attempt < OLED_INIT_RETRY_COUNT; ++attempt) {
        if (AppDebug_Init()) {
            return true;
        }
        if ((attempt + 1U) < OLED_INIT_RETRY_COUNT) {
            SystemTime_DelayMs(OLED_INIT_RETRY_DELAY_MS);
        }
    }
#else
    (void) attempt;
#endif
    return false;
}

static int32_t absolute_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void reset_runtime_counters(void)
{
    s_line_loss_ms = 0U;
    s_invalid_ms = 0U;
    s_curve_enter_ms = 0U;
    s_curve_exit_ms = 0U;
    s_free_line_ms = 0U;
    s_curve_candidate_side = 0;
    AppTurnDetector_Reset();
}

static bool command_output(const LineControlOutput *output)
{
    g_control_correction = output->correction;
    g_left_speed = output->left_speed;
    g_right_speed = output->right_speed;
    return Motor_SetChassisSpeed(output->left_speed, output->right_speed);
}

static void stop_vehicle(void)
{
    s_output.base_speed = 0;
    s_output.correction = 0;
    s_output.left_speed = 0;
    s_output.right_speed = 0;
    g_control_correction = 0;
    g_left_speed = 0;
    g_right_speed = 0;
    (void) Motor_Stop();
}

static void enter_stop(AppFault fault)
{
    stop_vehicle();
    g_app_state = APP_STATE_STOP;
    g_app_fault = fault;
    s_state_entry_ms = SystemTime_NowMs();
    reset_runtime_counters();
    StatusLed_Set(fault != APP_FAULT_NONE, fault == APP_FAULT_NONE);

    if (fault != APP_FAULT_NONE) {
        Buzzer_PlayLong(1U);
    }
    /* 车辆已停车，此时允许完整刷新故障和诊断数据。 */
    queue_oled_event(true);
}

static void enter_tracking(AppState state)
{
    g_app_state = state;
    g_app_fault = APP_FAULT_NONE;
    s_state_entry_ms = SystemTime_NowMs();
    reset_runtime_counters();
    AppLineControl_Reset(s_line.error);
    StatusLed_Set(false, true);

    if (state == APP_STATE_STRAIGHT_TRACKING) {
        Buzzer_PlayShort(1U);
    } else if (state == APP_STATE_FREE_STRAIGHT) {
        Buzzer_PlayShort(2U);
        s_free_heading_mdeg = Mpu6050_GetYawMdeg();
    }
    queue_oled_event(false);
}

static void enter_corner(TurnDirection direction)
{
    Mpu6050_ResetRelativeYaw();
    AppTurnControl_Begin(&s_turn, direction, SystemTime_NowMs());
    g_app_state = (direction == TURN_DIRECTION_LEFT) ?
        APP_STATE_LEFT_CORNER : APP_STATE_RIGHT_CORNER;
    g_app_fault = APP_FAULT_NONE;
    StatusLed_Set(true, true);
    Buzzer_PlayShort((direction == TURN_DIRECTION_LEFT) ? 4U : 5U);
    queue_oled_event(false);
}

static void update_diagnostics(void)
{
    g_line_raw = s_line.raw;
    g_line_error = s_line.error;
    g_yaw_mdeg = Mpu6050_GetYawMdeg();
    g_yaw_rate_mdps = Mpu6050_GetYawRateMdps();
}

static bool critical_modules_ready(void)
{
    return g_self_test_motor_ok && g_self_test_line_ok &&
        g_self_test_imu_ok;
}

static void handle_start_request(void)
{
    EightTracking_Read(&s_line);
    update_diagnostics();

    /*
     * 八路灰度使用普通GPIO采样，没有ACK或设备ID，任何0/1组合都可能是
     * 合法现场数据，不能用raw==0判断模块断线。具体图案是否合法由下面
     * 的valid_line/pattern分支负责：连续黑线直接进入状态1，全白进入
     * 状态2，离散或全黑图案进入FAULT2而非FAULT1。
     */
    g_self_test_line_ok = true;
    g_self_test_motor_ok = Motor_Probe();
    if (!critical_modules_ready()) {
        enter_stop(APP_FAULT_SELF_TEST);
        return;
    }

    if (s_line.valid_line) {
        enter_tracking(APP_STATE_STRAIGHT_TRACKING);
    } else if (s_line.pattern == LINE_PATTERN_ALL_WHITE) {
        enter_tracking(APP_STATE_FREE_STRAIGHT);
    } else {
        enter_stop(APP_FAULT_START_PATTERN);
    }
}

static bool handle_abnormal_line(void)
{
    if (s_line.pattern == LINE_PATTERN_ALL_WHITE) {
        s_invalid_ms = 0U;
        if (s_line_loss_ms < LINE_LONG_LOSS_MS) {
            s_line_loss_ms = (uint16_t) (s_line_loss_ms +
                APP_CONTROL_PERIOD_MS);
        }
        if (s_line_loss_ms >= LINE_LONG_LOSS_MS) {
            enter_stop(APP_FAULT_LINE_LOST);
            return false;
        }
        if (s_line_loss_ms > LINE_SHORT_LOSS_MS) {
            stop_vehicle();
            return false;
        }
        return command_output(&s_output);
    }

    s_line_loss_ms = 0U;
    if (s_invalid_ms < LINE_WIDE_INVALID_MS) {
        s_invalid_ms = (uint16_t) (s_invalid_ms + APP_CONTROL_PERIOD_MS);
    }
    if (s_invalid_ms >= LINE_WIDE_INVALID_MS) {
        enter_stop(APP_FAULT_LINE_INVALID);
        return false;
    }
    return command_output(&s_output);
}

static bool detect_and_enter_corner(void)
{
    TurnDirection direction;

    direction = AppTurnDetector_Update(&s_line,
        APP_CONTROL_PERIOD_MS, s_corner_cooldown_ms != 0U);
    if (direction == TURN_DIRECTION_NONE) {
        return false;
    }
    enter_corner(direction);
    return true;
}

static void run_line_tracking(LineControlProfile profile)
{
    if (detect_and_enter_corner()) {
        /* 同一控制周期立即给出第一条转弯指令。 */
        TurnResult ignored = AppTurnControl_Update(&s_turn, &s_line,
            Mpu6050_GetRelativeYawMdeg(), SystemTime_NowMs(), &s_output);
        (void) ignored;
        if (!command_output(&s_output)) {
            enter_stop(APP_FAULT_MOTOR_COMM);
        }
        return;
    }

    if (!s_line.valid_line) {
        if (!handle_abnormal_line() &&
            (g_app_state != APP_STATE_STOP)) {
            /* 尚未超时，仅保持停车等待下一次有效采样。 */
        }
        return;
    }

    s_line_loss_ms = 0U;
    s_invalid_ms = 0U;
    AppLineControl_Update(&s_line, profile,
        Mpu6050_GetYawRateMdps(), &s_output);
    if (!command_output(&s_output)) {
        enter_stop(APP_FAULT_MOTOR_COMM);
        return;
    }

    if (profile == LINE_CONTROL_STRAIGHT) {
        if (absolute_i32(s_line.error) >= CURVE_ENTER_ERROR_MIN) {
            const int8_t error_side = (s_line.error > 0) ? 1 : -1;

            /* 必须在同一侧持续偏离；回中或换边都会重新开始500 ms确认。 */
            if (error_side != s_curve_candidate_side) {
                s_curve_candidate_side = error_side;
                s_curve_enter_ms = APP_CONTROL_PERIOD_MS;
            } else {
                s_curve_enter_ms = (uint16_t) (s_curve_enter_ms +
                    APP_CONTROL_PERIOD_MS);
            }
            if (s_curve_enter_ms >= CURVE_ENTER_CONFIRM_MS) {
                g_app_state = APP_STATE_CURVE_TRACKING;
                s_state_entry_ms = SystemTime_NowMs();
                s_curve_exit_ms = 0U;
                s_curve_candidate_side = 0;
                AppLineControl_Reset(s_line.error);
                queue_oled_event(false);
            }
        } else {
            s_curve_enter_ms = 0U;
            s_curve_candidate_side = 0;
        }
    } else {
        /* S弯换向会短暂过零，必须低角速度并且直线灰度持续成立。 */
        if (s_line.straight_line &&
            (absolute_i32(Mpu6050_GetYawRateMdps()) <=
                CURVE_EXIT_RATE_MDPS)) {
            s_curve_exit_ms = (uint16_t) (s_curve_exit_ms +
                APP_CONTROL_PERIOD_MS);
            if (s_curve_exit_ms >= CURVE_EXIT_CONFIRM_MS) {
                enter_tracking(APP_STATE_STRAIGHT_TRACKING);
            }
        } else {
            s_curve_exit_ms = 0U;
        }
    }
}

static void run_free_straight(void)
{
    /*
     * 状态2只等待“连续且非全黑”的黑线，不要求必须落在中心探头，
     * 也不要求是窄直线；进入状态1后由正常加权误差和PD负责拉回线路。
     * 全白、离散和全黑仍不会触发切换。
     */
    if (s_line.valid_line) {
        s_free_line_ms = (uint16_t) (s_free_line_ms +
            APP_CONTROL_PERIOD_MS);
        if (s_free_line_ms >= FREE_RUN_LINE_CONFIRM_MS) {
            enter_tracking(APP_STATE_STRAIGHT_TRACKING);
            return;
        }
    } else {
        s_free_line_ms = 0U;
    }

    if (SystemTime_ElapsedMs(s_state_entry_ms) >= FREE_RUN_TIMEOUT_MS) {
        enter_stop(APP_FAULT_FREE_RUN_TIMEOUT);
        return;
    }

    /* 正值修正使车辆右转；Yaw正方向为左，因此使用当前值减目标值。 */
    AppLineControl_HeadingStraight(Mpu6050_GetYawMdeg() -
        s_free_heading_mdeg, &s_output);
    if (!command_output(&s_output)) {
        enter_stop(APP_FAULT_MOTOR_COMM);
    }
}

static void run_corner(void)
{
    const TurnResult result = AppTurnControl_Update(&s_turn, &s_line,
        Mpu6050_GetRelativeYawMdeg(), SystemTime_NowMs(), &s_output);

    if (result == TURN_RESULT_DONE) {
        stop_vehicle();
        s_corner_cooldown_ms = CORNER_COOLDOWN_MS;
        enter_tracking(APP_STATE_STRAIGHT_TRACKING);
        return;
    }
    if (result != TURN_RESULT_ACTIVE) {
        enter_stop((g_app_state == APP_STATE_LEFT_CORNER) ?
            APP_FAULT_LEFT_CORNER : APP_FAULT_RIGHT_CORNER);
        return;
    }
    if (!command_output(&s_output)) {
        enter_stop(APP_FAULT_MOTOR_COMM);
    }
}

static void control_step(void)
{
    const bool stationary = g_app_state == APP_STATE_STOP;

    EightTracking_Read(&s_line);
    if (!Mpu6050_Update(APP_CONTROL_PERIOD_MS, stationary)) {
        g_self_test_imu_ok = false;
        if (g_app_state != APP_STATE_STOP) {
            enter_stop(APP_FAULT_IMU);
        }
    }
    update_diagnostics();

    if (s_corner_cooldown_ms != 0U) {
        s_corner_cooldown_ms = (s_corner_cooldown_ms >
            APP_CONTROL_PERIOD_MS) ?
            (uint16_t) (s_corner_cooldown_ms - APP_CONTROL_PERIOD_MS) : 0U;
    }

    switch (g_app_state) {
        case APP_STATE_STOP:
            break;
        case APP_STATE_STRAIGHT_TRACKING:
            run_line_tracking(LINE_CONTROL_STRAIGHT);
            break;
        case APP_STATE_FREE_STRAIGHT:
            run_free_straight();
            break;
        case APP_STATE_CURVE_TRACKING:
            run_line_tracking(LINE_CONTROL_CURVE);
            break;
        case APP_STATE_LEFT_CORNER:
        case APP_STATE_RIGHT_CORNER:
            run_corner();
            break;
        default:
            enter_stop(APP_FAULT_SELF_TEST);
            break;
    }
}

static void refresh_debug(void)
{
    AppDebugSnapshot snapshot;

    snapshot.state = (uint8_t) g_app_state;
    snapshot.fault = (uint8_t) g_app_fault;
    snapshot.line_raw = g_line_raw;
    snapshot.line_error = g_line_error;
    snapshot.correction = g_control_correction;
    snapshot.left_speed = g_left_speed;
    snapshot.right_speed = g_right_speed;
    snapshot.yaw_mdeg = g_yaw_mdeg;
    snapshot.yaw_rate_mdps = g_yaw_rate_mdps;
    snapshot.motor_ok = g_self_test_motor_ok;
    snapshot.line_ok = g_self_test_line_ok;
    snapshot.imu_ok = g_self_test_imu_ok;
    snapshot.imu_status = (uint8_t) Mpu6050_GetStatus();
    snapshot.imu_who_am_i = g_mpu6050_who_am_i;
    snapshot.oled_ok = g_self_test_oled_ok;
    AppDebug_Refresh(&snapshot);
}

void AppStateMachine_Init(void)
{
    /* OLED先完成上电自检并显示启动页，随后才进入其余模块自检。 */
    g_self_test_oled_ok = init_oled_with_retry();
    AppDebug_ShowBoot("SELF TEST");

    Motor_Init();
    EightTracking_Init();
    Key_Init();
    StatusLed_Set(true, false);

    /* 无论自检成功与否，车辆首先保持停车。 */
    g_self_test_motor_ok = Motor_Stop() && Motor_Probe();
    EightTracking_Read(&s_line);
    /* GPIO型灰度模块无法通过单次电平组合判断是否在线。 */
    g_self_test_line_ok = true;

    g_self_test_imu_ok = Mpu6050_Init();
    if (g_self_test_imu_ok) {
        /* 这条提示必须在阻塞式2秒预热/400次采样之前显示。 */
        AppDebug_ShowBoot("IMU WARMUP");
        g_self_test_imu_ok = Mpu6050_Calibrate();
    }

    update_diagnostics();
    s_last_key_ms = SystemTime_NowMs();
    s_last_control_ms = s_last_key_ms;
    s_corner_cooldown_ms = 0U;
    s_oled_event_pending = false;
    s_oled_full_refresh_pending = false;

    if (critical_modules_ready()) {
        enter_stop(APP_FAULT_NONE);
        Buzzer_PlayShort(1U);
    } else {
        enter_stop(APP_FAULT_SELF_TEST);
    }
}

void AppStateMachine_Run(void)
{
    const uint32_t now = SystemTime_NowMs();

    if ((uint32_t) (now - s_last_key_ms) >= APP_KEY_PERIOD_MS) {
        bool k1_pressed;
        bool k4_pressed;

        s_last_key_ms += APP_KEY_PERIOD_MS;
        Key_Update();
        k1_pressed = Key_TakePressed(KEY_K1);
        k4_pressed = Key_TakePressed(KEY_K4);

        if ((g_app_state != APP_STATE_STOP) && k4_pressed) {
            enter_stop(APP_FAULT_EMERGENCY_STOP);
        }
        if ((g_app_state == APP_STATE_STOP) && k1_pressed) {
            handle_start_request();
        }
    }

    if ((uint32_t) (now - s_last_control_ms) >= APP_CONTROL_PERIOD_MS) {
        s_last_control_ms += APP_CONTROL_PERIOD_MS;
        control_step();
    }

    /* 只处理状态变化或故障事件，不再每100 ms刷新整屏。 */
    service_oled_event();
}
