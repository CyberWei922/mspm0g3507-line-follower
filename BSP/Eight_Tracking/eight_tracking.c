#include "eight_tracking.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define ADDRESS_PINS (LINE_GPIO_AD0_PIN | LINE_GPIO_AD1_PIN | \
                      LINE_GPIO_AD2_PIN)
#define CENTER_MASK  (0x18U)
#define START_ZONE_MASK (0x3CU) /* X3～X6 */

static const int8_t s_weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static void select_channel(uint8_t channel)
{
    uint32_t high = 0U;

    if ((channel & 0x01U) != 0U) {
        high |= LINE_GPIO_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U) {
        high |= LINE_GPIO_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U) {
        high |= LINE_GPIO_AD2_PIN;
    }
    DL_GPIO_clearPins(LINE_GPIO_PORT, ADDRESS_PINS);
    if (high != 0U) {
        DL_GPIO_setPins(LINE_GPIO_PORT, high);
    }
    delay_cycles((CPUCLK_FREQ / 1000000U) * LINE_CHANNEL_SETTLE_US);
}

static uint8_t scan_once(void)
{
    uint8_t raw = 0U;
    uint8_t channel;

    for (channel = 0U; channel < 8U; ++channel) {
        select_channel(channel);
        if ((DL_GPIO_readPins(LINE_GPIO_PORT, LINE_GPIO_OUT_PIN) &
                LINE_GPIO_OUT_PIN) != 0U) {
            raw |= (uint8_t) (1U << channel);
        }
    }
    return raw;
}

/*
 * 根据你给出的逐探头测试现象（第三个探头显示到第五位），这块实际模块
 * 的OUT扫描位与探头编号不是一一同序：扫描2/3落在X5/X6，扫描4/5
 * 落在X3/X4。PA14/15/16/17的物理接线定义不改，只在读取后把扫描位
 * 还原成X1..X8逻辑位，后面的PID和弯道检测均使用还原值。
 */
static uint8_t restore_sensor_order(uint8_t scanned)
{
    uint8_t restored = scanned & 0xC3U;

    if ((scanned & (1U << 2U)) != 0U) {
        restored |= (1U << 4U);
    }
    if ((scanned & (1U << 3U)) != 0U) {
        restored |= (1U << 5U);
    }
    if ((scanned & (1U << 4U)) != 0U) {
        restored |= (1U << 2U);
    }
    if ((scanned & (1U << 5U)) != 0U) {
        restored |= (1U << 3U);
    }
    return restored;
}

static uint8_t count_bits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t) (count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static bool bits_are_contiguous(uint8_t mask)
{
    if (mask == 0U) {
        return false;
    }
    while ((mask & 1U) == 0U) {
        mask >>= 1U;
    }
    while ((mask & 1U) != 0U) {
        mask >>= 1U;
    }
    return mask == 0U;
}

void EightTracking_Init(void)
{
    DL_GPIO_clearPins(LINE_GPIO_PORT, ADDRESS_PINS);
}

uint8_t EightTracking_ReadRaw(void)
{
    const uint8_t first = scan_once();
    const uint8_t second = scan_once();
    const uint8_t third = scan_once();

    /* 三次完整扫描逐位多数表决，过滤一次反光或跳变。 */
    return restore_sensor_order((uint8_t) ((first & second) |
        (first & third) | (second & third)));
}

void EightTracking_Read(LineObservation *observation)
{
    uint8_t channel;
    int16_t weighted_sum = 0;
    uint8_t physical_channel;

    if (observation == 0) {
        return;
    }

    observation->raw = EightTracking_ReadRaw();
#if LINE_BLACK_IS_LOW
    observation->black_mask = (uint8_t) ~observation->raw;
#else
    observation->black_mask = observation->raw;
#endif
    observation->active_count = count_bits(observation->black_mask);
    observation->contiguous = bits_are_contiguous(observation->black_mask);
    observation->error = 0;
    observation->valid_line = false;
    observation->straight_line = false;
    observation->start_line_valid = false;

    if (observation->black_mask == 0U) {
        observation->pattern = LINE_PATTERN_ALL_WHITE;
        return;
    }
    if (observation->black_mask == 0xFFU) {
        observation->pattern = LINE_PATTERN_ALL_BLACK;
        return;
    }
    if (!observation->contiguous) {
        observation->pattern = LINE_PATTERN_DISCRETE_INVALID;
        return;
    }
    if (observation->active_count > 4U) {
        observation->pattern = LINE_PATTERN_TOO_WIDE;
        return;
    }

    for (channel = 0U; channel < 8U; ++channel) {
        if ((observation->black_mask & (uint8_t) (1U << channel)) != 0U) {
#if LINE_CHANNEL0_IS_LEFT
            physical_channel = channel;
#else
            physical_channel = (uint8_t) (7U - channel);
#endif
            weighted_sum += s_weights[physical_channel];
        }
    }

    observation->error =
        (int16_t) (weighted_sum / observation->active_count);
    observation->pattern = LINE_PATTERN_VALID;
    observation->valid_line = true;
    observation->straight_line =
        ((observation->black_mask & CENTER_MASK) != 0U) &&
        (observation->active_count <= 3U) &&
        (observation->error >= -2) && (observation->error <= 2);
    observation->start_line_valid = observation->straight_line &&
        ((observation->black_mask & START_ZONE_MASK) != 0U);
}
