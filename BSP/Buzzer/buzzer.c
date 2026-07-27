#include "buzzer.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

typedef enum {
    BUZZER_PHASE_IDLE = 0,
    BUZZER_PHASE_ON,
    BUZZER_PHASE_GAP
} BuzzerPhase;

static volatile BuzzerPhase s_phase;
static volatile uint16_t s_remaining_ms;
static volatile uint16_t s_on_ms;
static volatile uint8_t s_remaining_beeps;

static void set_output(bool enabled)
{
    /* 100计数向下PWM：比较值99近似关闭，30产生清晰蜂鸣。 */
    DL_TimerA_setCaptureCompareValue(BUZZER_INST,
        enabled ? 30U : 99U, GPIO_BUZZER_C3_IDX);
}

static void start_pattern(uint8_t count, uint16_t on_ms)
{
    if (count == 0U) {
        Buzzer_Stop();
        return;
    }
    s_remaining_beeps = count;
    s_on_ms = on_ms;
    s_remaining_ms = on_ms;
    s_phase = BUZZER_PHASE_ON;
    set_output(true);
}

void Buzzer_Init(void)
{
    s_phase = BUZZER_PHASE_IDLE;
    set_output(false);
    DL_Timer_startCounter(BUZZER_INST);
}

void Buzzer_PlayShort(uint8_t count)
{
    start_pattern(count, BUZZER_SHORT_MS);
}

void Buzzer_PlayLong(uint8_t count)
{
    start_pattern(count, BUZZER_LONG_MS);
}

void Buzzer_Stop(void)
{
    s_phase = BUZZER_PHASE_IDLE;
    s_remaining_ms = 0U;
    s_remaining_beeps = 0U;
    set_output(false);
}

bool Buzzer_IsBusy(void)
{
    return s_phase != BUZZER_PHASE_IDLE;
}

void Buzzer_Tick1ms(void)
{
    if (s_phase == BUZZER_PHASE_IDLE) {
        return;
    }
    if (s_remaining_ms != 0U) {
        --s_remaining_ms;
        return;
    }

    if (s_phase == BUZZER_PHASE_ON) {
        set_output(false);
        if (--s_remaining_beeps == 0U) {
            s_phase = BUZZER_PHASE_IDLE;
        } else {
            s_phase = BUZZER_PHASE_GAP;
            s_remaining_ms = BUZZER_GAP_MS;
        }
    } else {
        set_output(true);
        s_phase = BUZZER_PHASE_ON;
        s_remaining_ms = s_on_ms;
    }
}
