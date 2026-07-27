#include "app_debug.h"

#include "app_config.h"
#include "BSP/OLED/oled.h"

static bool s_available;

static char *append_text(char *destination, const char *text)
{
    while (*text != '\0') {
        *destination++ = *text++;
    }
    return destination;
}

static char *append_i32(char *destination, int32_t value)
{
    char reverse[11];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (value < 0) {
        *destination++ = '-';
        magnitude = (uint32_t) (-(int64_t) value);
    } else {
        magnitude = (uint32_t) value;
    }
    do {
        reverse[count++] = (char) ('0' + magnitude % 10U);
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(reverse)));
    while (count != 0U) {
        *destination++ = reverse[--count];
    }
    return destination;
}

static void show_pair(uint8_t page, const char *left_label, int32_t left,
    const char *right_label, int32_t right)
{
    char text[22];
    char *cursor = text;

    cursor = append_text(cursor, left_label);
    cursor = append_i32(cursor, left);
    *cursor++ = ' ';
    cursor = append_text(cursor, right_label);
    cursor = append_i32(cursor, right);
    *cursor = '\0';
    Oled_ShowString(0U, page, text);
}

static void show_triplet(uint8_t page, const char *first_label, int32_t first,
    const char *second_label, int32_t second, const char *third_label,
    int32_t third)
{
    char text[22];
    char *cursor = text;

    cursor = append_text(cursor, first_label);
    cursor = append_i32(cursor, first);
    *cursor++ = ' ';
    cursor = append_text(cursor, second_label);
    cursor = append_i32(cursor, second);
    *cursor++ = ' ';
    cursor = append_text(cursor, third_label);
    cursor = append_i32(cursor, third);
    *cursor = '\0';
    Oled_ShowString(0U, page, text);
}

static void show_line_bits(uint8_t page, uint8_t raw)
{
    /* 保持之前验证版的bit0..bit7显示顺序，避免改变调试数据含义。 */
    char text[15] = "RAW:";
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit) {
        text[4U + bit] = ((raw & (uint8_t) (1U << bit)) != 0U) ?
            '1' : '0';
    }
    text[12] = '\0';
    Oled_ShowString(0U, page, text);
}

bool AppDebug_Init(void)
{
#if APP_ENABLE_OLED
    s_available = Oled_Init();
#else
    s_available = false;
#endif
    return s_available;
}

void AppDebug_ShowBoot(const char *message)
{
    if (!s_available) {
        return;
    }
    Oled_Clear();
    Oled_ShowString(0U, 0U, "LINE TRACKER");
    Oled_ShowString(0U, 2U, message);
    (void) Oled_Refresh();
}

void AppDebug_Refresh(const AppDebugSnapshot *snapshot)
{
    char state_text[22];
    char *cursor;

    if (!s_available || (snapshot == 0)) {
        return;
    }
    Oled_Clear();

    cursor = append_text(state_text, "STATE:");
    cursor = append_i32(cursor, snapshot->state);
    cursor = append_text(cursor, " FAULT:");
    cursor = append_i32(cursor, snapshot->fault);
    *cursor = '\0';
    Oled_ShowString(0U, 0U, state_text);
    show_line_bits(1U, snapshot->line_raw);
    show_pair(2U, "ERR:", snapshot->line_error,
        "PD:", snapshot->correction);
    show_pair(3U, "L:", snapshot->left_speed,
        "R:", snapshot->right_speed);
    show_pair(4U, "YAW:", snapshot->yaw_mdeg / 100,
        "", 0);
    show_pair(5U, "RATE:", snapshot->yaw_rate_mdps / 100,
        "", 0);
    show_triplet(6U, "M:", snapshot->motor_ok ? 1 : 0,
        "L:", snapshot->line_ok ? 1 : 0,
        "O:", snapshot->oled_ok ? 1 : 0);
    show_triplet(7U, "I:", snapshot->imu_ok ? 1 : 0,
        "S:", snapshot->imu_status,
        "W:", snapshot->imu_who_am_i);

    if (!Oled_Refresh()) {
        s_available = false;
    }
}

bool AppDebug_IsAvailable(void)
{
    return s_available;
}
