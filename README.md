# Yahboom 8-channel black-line PID follower

This is a separate MSPM0G3507 project. It does not modify
`yahboom_motor_i2c_probe` or `yahboom_gray8_buzzer_probe`.

## Wiring

| Function | MSPM0G3507 |
| --- | --- |
| Motor SCL | PA12 |
| Motor SDA | PA13 |
| Grayscale OUT | PA14 |
| Grayscale AD0 | PA15 |
| Grayscale AD1 | PA16 |
| Grayscale AD2 | PA17 |
| Expansion-board buzzer | PB24 / TIMA0 CCP3 |

Use the same wiring that passed the grayscale/buzzer validation. During
runtime testing, power the car from its battery and avoid parallel USB power
paths through the core board or motor driver.

## Safe start sequence

1. Programming, debugger reset, and power-on always command zero motor speed.
2. One short beep means the program is idle and waiting for the physical RST
   button.
3. Put the sensor over a normal section of the 18 mm black tape, with the line
   roughly centered, then press the physical RST button.
4. The green LED flashes for two seconds while the car remains stopped.
5. Two short beeps mean the line was recognized and following is starting.
6. The car follows using differential steering. The initial base speed is 190.

## PID and right-angle behavior

Normal tracking keeps the already tested Kp/Kd values, with two robustness
changes:

- each of the eight digital sensors uses a three-scan majority vote;
- the derivative term is low-pass filtered, so a single sensor-edge spike does
  not create a large steering kick;
- tracking commands change by at most 35 units per control loop;
- on high-friction cloth, both sides keep at least speed 120 instead of
  stopping the inner wheels and dragging them sideways.

A confirmed right-angle pattern enters this non-blocking state sequence:

1. The same left/right corner must be detected for two consecutive loops.
2. Both sides drive at speed 150 for eight loops (about 160 ms). This moves the
   wheel rotation center toward the corner instead of pivoting as soon as the
   front sensor bar sees it.
3. The chassis pivots in place at speed 195 toward the detected side.
4. The old center line must first disappear and the pivot must last at least
   ten loops (about 200 ms). A valid 1-to-3-sensor line must then remain within
   75 position units of center for four consecutive loops.
5. The car uses reduced-correction PID at speed 155 for eight loops (about
   160 ms), resets PID history, and resumes normal tracking. If the line
   disappears during this phase, it returns to pivot/reacquisition.

No MPU6050 input is used in this version. The turn is closed-loop with the
eight-channel grayscale sensor rather than a fixed rotation time.

## Fault sounds

| Sound | Meaning |
| --- | --- |
| 3 long beeps | Motor driver did not acknowledge I2C |
| 4 short beeps | No valid 1-to-4-sensor line was present at startup |
| 2 long beeps | Line was lost for about 0.8 seconds |
| 3 short beeps | Six or more sensors stayed black for about 0.5 seconds |
| 5 short beeps | Corner pivot did not reacquire the line in about 1.2 seconds |

All motion faults send two zero-speed commands before sounding the buzzer.

## Initial control settings

- Approximate control period: 20 ms
- Normal base speed: 190
- Minimum normal-corner base speed: 165
- Minimum tracking wheel speed on cloth: 120
- Maximum wheel command: 320
- PID: Kp = 0.45, Ki = 0, Kd = 0.70
- Maximum steering correction: 145
- Per-loop command slew limit: 35
- Right-angle forward compensation: about 160 ms at speed 150
- Right-angle pivot speed: 195
- Minimum right-angle pivot time: about 200 ms
- Post-turn PID settling: about 160 ms at speed 155

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

1. If the car pivots too early or late, adjust `CORNER_APPROACH_CYCLES` by one
   loop (about 20 ms).
2. If it cannot rotate against floor friction or overshoots badly, adjust
   `CORNER_PIVOT_SPEED` in steps of 10.
3. Only after those are stable should PID gains be changed.

Channel 0 is initially treated as the physical left side. If the first
raised-wheel test proves that steering correction is reversed, change
`GRAY8_CHANNEL0_IS_LEFT` from `1U` to `0U`, rebuild, and retest. Do not test
this for the first time at full speed on the floor.
