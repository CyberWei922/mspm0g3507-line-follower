# Yahboom 8-channel black-line PID line follower

## Firmware version

- Version: `1.1.1-dev`
- Git tag: `line-follower-v1.1.1-dev`
- Status: local development build based on the published `mpu6050-v1.0.0`
  baseline

This project continues the Git history of `yahboom_black_line_pid` and adds the official
`Competition_PJ/BSP/MPU6050` + MotionApps/DMP driver through a local wrapper.

## Wiring

| Function | MSPM0G3507 |
| --- | --- |
| Motor SCL | PA12 |
| Motor SDA | PA13 |
| Grayscale OUT | PA14 |
| Grayscale AD0 | PA15 |
| Grayscale AD1 | PA16 |
| Grayscale AD2 | PA17 |
| MPU6050 SCL | PA1 |
| MPU6050 SDA | PA0 |
| MPU6050 AD0 | GND |
| MPU6050 power | 3V3 and GND |
| K1 | PA2 |
| K2 | PB19 |
| K3 | PB20 |
| K4 | PA23 |
| Expansion-board buzzer | PB24 / TIMA0 CCP3 |

MPU6050 `INT`, `XDA`, and `XCL` remain disconnected. The actual installation
used by this build is component-side up, with the vehicle nose pointing toward
the left side of the supplied module photograph. In that photograph sensor
`+X` points right and `+Y` points up, so sensor `+X` is vehicle rear, sensor
`+Y` is vehicle right, and sensor `+Z` points upward. The copied DMP
orientation matrix therefore applies a 180-degree rotation around Z. Vehicle
left yaw is positive and vehicle right yaw is negative.

Use the same wiring that passed the grayscale/buzzer validation. During
runtime testing, power the car from its battery and avoid parallel USB power
paths through the core board or motor driver.

## Button state machine and safe start sequence

1. Programming, debugger reset, and power-on always command zero motor speed.
2. The blue LED stays on while the MPU6050 is initialized, then the vehicle
   remains still for a 2-second warm-up and 400 stationary gyro-Z samples.
3. Two short beeps and a blue LED mean `READY`. Place the sensor over a normal
   section of the 18 mm black tape before pressing K1.
4. K1 starts or resumes tracking. K1 during motion pauses immediately and
   commands zero speed; K1 from `PAUSED` rechecks the line and resumes.
5. K2 lowers the base speed and K3 raises it, in 20-step increments while
   `READY` or `PAUSED`. The allowed range is 240..400.
6. K4 is an emergency stop. After it is released, K1 clears the stop and
   returns to `READY`.
7. The car follows using differential steering. The default base speed is 320.

## Experimental official LineWalking adaptation

Normal tracking is currently a local, uncommitted experiment adapted from the
Yahboom `Competition_PJ/BSP/Eight_Tracking` implementation:

- each of the eight digital sensors uses a three-scan majority vote;
- the local `OUT + AD0/AD1/AD2` scan is converted into the official
  `x1` through `x8` convention, where zero means black;
- the official priority table maps recognized patterns to errors
  `-3, -2, -1, 0, 1, 2, 3`, with `-15/+15` for sharp turns;
- unlisted patterns retain the previous error, reproducing the reference
  controller's direction memory;
- normal pattern-table PID uses Kp=150, Ki=4, Kd=0.5 and the adjustable base
  speed (default 320);
- the bounded grayscale-error integral handles persistent -1/+1 offsets that
  a yaw-rate-only controller cannot remove; it decays around the center and
  resets for sharp turns;
- the official `Motion_Car_Control(V_x, 0, V_z)` geometry conversion is kept,
  but its output is routed through the verified local motor signs and PA12/PA13
  software-I2C driver;
- the reference bug that never updated `error_last` is corrected.

A confirmed right-angle pattern enters this non-blocking state sequence:

1. The same left/right corner must be detected for two consecutive loops.
2. Both sides drive at speed 210 for eight loops (about 160 ms). This moves the
   wheel rotation center toward the corner instead of pivoting as soon as the
   front sensor bar sees it.
3. At the end of forward compensation, the current unwrapped DMP yaw is saved
   as the corner start angle.
4. The chassis pivots in place at speed 330. At 72 degrees it reduces pivot
   speed to 270 so the motors retain torque against cloth friction.
5. The old center line must first disappear. After at least 78 degrees, a
   valid 1-to-3-sensor line must remain within 75 position units of center for
   four consecutive loops.
6. More than 145 degrees without reacquisition stops the car. The time-based
   timeout is extended to 120 control cycles as a second guard.
7. The car uses reduced-correction PID at speed 180 for eight loops (about
   160 ms), resets PID history, and resumes normal tracking. If the line
   disappears during this phase, it returns to pivot/reacquisition.

Normal `-3..+3` patterns also receive a limited gyro-Z rate correction. The
requested yaw rate is filtered before use, and the measured-minus-requested
rate error supplies true negative feedback for the verified motor steering
sign. Centered-line damping is stronger than turning damping, reducing
left-right hunting without preventing the optical controller from entering a
curve. The official pattern-table result stays primary; the IMU contribution
is clamped to avoid fighting sharp-turn handling.

## Fault sounds

| Sound | Meaning |
| --- | --- |
| 3 long beeps | Motor driver did not acknowledge I2C |
| 4 short beeps | No valid 1-to-4-sensor line was present at startup |
| 2 long beeps | Line was lost for about 0.8 seconds |
| 3 short beeps | Six or more sensors stayed black for about 0.5 seconds |
| 5 short beeps | Corner pivot did not reacquire the line in about 1.6 seconds |
| 6 short beeps | MPU6050 ID, DMP, gyro read, or yaw stream failure |

All motion faults send two zero-speed commands before sounding the buzzer.

## Initial control settings

- Approximate control period: 15 ms (5 ms state tick, one control update every
  three ticks)
- Official pattern-table base speed: 320 (adjustable 240..400 with K2/K3)
- Official normal PID: Kp = 150, Ki = 4, Kd = 0.5
- Bounded grayscale-error integral: -80 to +80
- Official small pattern error range: -3 to +3
- Official sharp-turn error: -15 or +15
- Official `Motion_Car_Control` APB parameter: 188
- Pattern-controller signed wheel limit: -500 to +500
- Corner-settling PID: Kp = 0.60, Ki = 0, Kd = 0.55
- Right-angle forward compensation: about 160 ms at speed 210
- Right-angle pivot speed: 330, reduced to 270 after 72 degrees
- New-line acceptance begins at 78 degrees
- Maximum allowed corner angle: 145 degrees
- Post-turn PID settling: about 160 ms at speed 230
- MPU6050 DMP output: 50 Hz
- MPU6050 DLPF: explicitly restored and verified at 42 Hz after DMP setup
- MPU6050 yaw jump limit: 30° during normal tracking, temporarily 90° while
  inside the corner approach/pivot/settle state, then restored to 30°
- Gyro-Z startup calibration: 400 stationary samples after a 2-second warm-up
- Gyro-Z rate filter: scalar 1-D Kalman update; this reduces rate noise but
  does not create an absolute yaw reference
- Stationary zero-rate update: after about 0.5 seconds below 0.8 dps, bias
  adapts slowly and DMP yaw deltas are held rather than accumulated

The verified motor order is:

| Driver channel | Wheel | Forward command sign |
| --- | --- | --- |
| M1 | Left front | Positive |
| M2 | Right front | Negative |
| M3 | Left rear | Negative |
| M4 | Right rear | Positive |

Left/right differential steering therefore groups M1 with M3 and M2 with M4.

The integral path and anti-windup are implemented, but Ki starts at zero for
safe first tuning. PID constants are grouped at the top of the C source.

For first right-angle tuning, change only one setting at a time:

1. If the rotation slows too early or too late, adjust
   `CORNER_SLOW_ANGLE_DEGREES`.
2. If the car reaches the corner too early or late before starting rotation,
   adjust `CORNER_APPROACH_CYCLES`.
3. If it stops before the sensor reaches the new line, adjust
   `CORNER_REACQUIRE_ANGLE_DEGREES` only after checking actual logged yaw.
4. If it cannot rotate against floor friction or overshoots badly, adjust
   `CORNER_PIVOT_SPEED` and `CORNER_PIVOT_SLOW_SPEED` in steps of 10.
5. Only after angle handling is stable should gyro-rate feedback or line-PD
   gains be changed.

## First hardware validation

Before placing the car on the floor:

1. Raise all wheels and keep the module motionless during initialization.
2. Confirm there are two short start beeps, not six fault beeps.
3. This build assumes the confirmed component-side-up installation with the
   vehicle nose toward the photograph's left side.
4. During the first raised-wheel run, verify that a detected left corner makes
   the chassis pivot left and a detected right corner makes it pivot right.
5. If the first floor run amplifies left-right oscillation instead of damping
   it, stop immediately; do not keep running at speed. The installation
   assumption or a gyro sign must be corrected before the next test.

The build uses a 2 KB stack because the official DMP quaternion conversion has
substantially more local state than the original follower.

Channel 0 is initially treated as the physical left side. If the first
raised-wheel test proves that steering correction is reversed, change
`GRAY8_CHANNEL0_IS_LEFT` from `1U` to `0U`, rebuild, and retest. Do not test
this for the first time at full speed on the floor.
