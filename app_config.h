#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/*
 * 全部可调参数集中在这里。首版优先复现参考工程的稳态表现，
 * 实车验证前不要同时修改多组参数。
 */

#define APP_ENABLE_OLED                    (1U)

/* OLED模块上电后先等待电源和控制器稳定，再进行第一次I2C访问。 */
#define OLED_POWER_ON_SETTLE_MS            (200U)
#define OLED_INIT_RETRY_COUNT              (4U)
#define OLED_INIT_RETRY_DELAY_MS           (50U)

/*
 * 临时静音开关：只断开PB24上的PWM物理输出，不改蜂鸣器状态机和提示规则。
 * 室内需要恢复声音时改回1，再重新编译即可。
 */
#define APP_ENABLE_BUZZER_OUTPUT           (0U)

/* 裸机任务周期。OLED改为事件触发，不再周期刷新。 */
#define APP_KEY_PERIOD_MS                  (5U)
#define APP_CONTROL_PERIOD_MS              (20U)

/* 八路灰度：沿用之前实车验证版本的物理方向，通道0按车体左侧处理。 */
#define LINE_CHANNEL0_IS_LEFT              (1U)
#define LINE_BLACK_IS_LOW                  (0U)
#define LINE_SCAN_MAJORITY_COUNT           (3U)
#define LINE_CHANNEL_SETTLE_US             (80U)

/* 参考稳定工程的普通循迹速度，单位沿用电机板速度指令。 */
#define LINE_SPEED_STRAIGHT                (200)
#define LINE_SPEED_CURVE                   (160)
#define LINE_SPEED_EDGE                    (135)
#define LINE_SPEED_OUTER                   (110)
#define LINE_WHEEL_SPEED_MAX               (360)

/* 状态1：直线循迹，采用真正的PD；第一版关闭积分。 */
#define LINE_STRAIGHT_KP                   (18)
#define LINE_STRAIGHT_KD                   (8)
#define LINE_STRAIGHT_MAX_CORRECTION       (160)
#define LINE_STRAIGHT_GYRO_DAMP_DEN        (2)  /* 约0.5*角速度 */
/* X3+X4或X5+X6亮时追加修正，尽早把黑线拉回X4+X5。 */
#define LINE_STRAIGHT_NEAR_CENTER_BOOST    (28)

/* 状态3：弯道更贴线，降低中心速度并减弱陀螺仪阻尼。 */
#define LINE_CURVE_KP                      (20)
#define LINE_CURVE_KD                      (6)
#define LINE_CURVE_CENTER_SPEED            (160)
#define LINE_CURVE_MAX_CORRECTION          (175)
#define LINE_CURVE_GYRO_DAMP_DEN           (4)

/* 普通状态的短时丢线记忆和最终停车限制。 */
#define LINE_SHORT_LOSS_MS                 (60U)
#define LINE_LONG_LOSS_MS                  (300U)
#define LINE_WIDE_INVALID_MS               (120U)

/* 状态1/3切换：X2+X3、X6+X7或更大同向偏离持续500 ms才进入状态3。 */
#define CURVE_ENTER_ERROR_MIN              (4)
#define CURVE_EXIT_RATE_MDPS               (6000L)
#define CURVE_ENTER_CONFIRM_MS             (500U)
#define CURVE_EXIT_CONFIRM_MS              (240U)

/* 状态2：无黑线起步时按进入状态时的航向自主直行。 */
#define FREE_RUN_SPEED                     (160)
#define FREE_RUN_HEADING_KP_NUM            (3)
#define FREE_RUN_HEADING_KP_DEN            (1000) /* 输入单位为mdeg */
#define FREE_RUN_MAX_CORRECTION            (80)
#define FREE_RUN_LINE_CONFIRM_MS           (60U)
#define FREE_RUN_TIMEOUT_MS                (30000U)

/* 直角识别、连续保线转弯、短时丢线搜索和安全限制。 */
#define CORNER_PRIMARY_CONFIRM_MS          (20U)
#define CORNER_EDGE_CONFIRM_MS             (40U)
/*
 * 直角弯控制移植自“陀螺仪接入稳态循迹转弯/project”：左右轮等幅
 * 反向旋转，离开旧中心线后，新线连续两次进入中心即退出。
 */
#define CORNER_PIVOT_SPEED                 (125)
#define CORNER_MIN_TURN_MS                 (100U)
#define CORNER_LEAVE_MIN_ANGLE_MDEG        (18000L)
#define CORNER_REACQUIRE_CONFIRM_MS        (40U)
#define CORNER_TIMEOUT_MS                  (12000U)
#define CORNER_MAX_ANGLE_MDEG               (360000L)
#define CORNER_COOLDOWN_MS                 (300U)

/* MPU6050：2秒预热、400次零偏、42 Hz DLPF、200 Hz内部采样。 */
#define IMU_WARMUP_MS                      (2000U)
#define IMU_CALIBRATION_SAMPLES            (400U)
#define IMU_CALIBRATION_PERIOD_MS          (5U)
#define IMU_YAW_SIGN                       (1)
#define IMU_YAW_SCALE_NUM                  (1000L)
#define IMU_YAW_SCALE_DEN                  (1000L)
#define IMU_RATE_DEADBAND_MDPS             (300L)
#define IMU_STATIONARY_RATE_MDPS           (800L)
#define IMU_STATIONARY_CONFIRM_MS          (500U)
#define IMU_ZERO_PULL_LIMIT_MDEG           (6000L)

/* 按键和蜂鸣器。 */
#define KEY_DEBOUNCE_MS                    (25U)
#define BUZZER_SHORT_MS                    (80U)
#define BUZZER_LONG_MS                     (500U)
#define BUZZER_GAP_MS                      (100U)

#endif /* APP_CONFIG_H_ */
