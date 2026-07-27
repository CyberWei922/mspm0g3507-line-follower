#include "oled.h"

#include "ti_msp_dl_config.h"

#define OLED_WIDTH        (128U)
#define OLED_PAGE_COUNT   (8U)
#define OLED_BUFFER_SIZE  (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_ADDRESS_0    (0x3CU)
#define OLED_ADDRESS_1    (0x3DU)
#define OLED_PACKET_MAX   (8U)
#define OLED_TIMEOUT      (CPUCLK_FREQ / 20U)

static uint8_t s_buffer[OLED_BUFFER_SIZE];
static uint8_t s_address;

static const uint8_t s_digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t s_upper[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
};

static bool wait_idle(void)
{
    uint32_t timeout = OLED_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    return true;
}

static bool write_packet(uint8_t address, const uint8_t *packet,
    uint8_t length)
{
    uint32_t timeout;
    uint32_t status;

    if ((packet == 0) || (length == 0U) || (length > OLED_PACKET_MAX) ||
        !wait_idle()) {
        return false;
    }
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, packet, length) !=
            length) {
        return false;
    }
    DL_I2C_startControllerTransfer(OLED_I2C_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles((CPUCLK_FREQ / 1000000U) * 4U);

    timeout = OLED_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout-- == 0U) {
            DL_I2C_resetControllerTransfer(OLED_I2C_INST);
            return false;
        }
    }
    status = DL_I2C_getControllerStatus(OLED_I2C_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(OLED_I2C_INST);
        DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
        return false;
    }
    return true;
}

static bool write_commands(const uint8_t *commands, uint8_t count)
{
    uint8_t packet[OLED_PACKET_MAX];
    uint8_t index = 0U;

    while (index < count) {
        uint8_t copied = (uint8_t) (count - index);
        uint8_t i;

        if (copied > 7U) {
            copied = 7U;
        }
        packet[0] = 0x00U;
        for (i = 0U; i < copied; ++i) {
            packet[i + 1U] = commands[index + i];
        }
        if (!write_packet(s_address, packet, (uint8_t) (copied + 1U))) {
            return false;
        }
        index = (uint8_t) (index + copied);
    }
    return true;
}

static const uint8_t *glyph(char character)
{
    static const uint8_t space[5] = {0,0,0,0,0};
    static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t question[5] = {0x02,0x01,0x51,0x09,0x06};

    if ((character >= 'a') && (character <= 'z')) {
        character = (char) (character - 'a' + 'A');
    }
    if ((character >= '0') && (character <= '9')) {
        return s_digits[(uint8_t) (character - '0')];
    }
    if ((character >= 'A') && (character <= 'Z')) {
        return s_upper[(uint8_t) (character - 'A')];
    }
    switch (character) {
        case ' ': return space;
        case '+': return plus;
        case '-': return minus;
        case '.': return dot;
        case ':': return colon;
        default: return question;
    }
}

static void draw_char(uint8_t x, uint8_t page, char character)
{
    const uint8_t *font;
    uint16_t offset;
    uint8_t column;

    if ((page >= OLED_PAGE_COUNT) || (x > OLED_WIDTH - 6U)) {
        return;
    }
    font = glyph(character);
    offset = (uint16_t) page * OLED_WIDTH + x;
    for (column = 0U; column < 5U; ++column) {
        s_buffer[offset + column] = font[column];
    }
    s_buffer[offset + 5U] = 0U;
}

bool Oled_Init(void)
{
    static const uint8_t init[] = {
        0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
        0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,
        0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0x2E,0xAF
    };
    const uint8_t probe[2] = {0x00U, 0xAEU};

    s_address = OLED_ADDRESS_0;
    if (!write_packet(s_address, probe, sizeof(probe))) {
        s_address = OLED_ADDRESS_1;
        if (!write_packet(s_address, probe, sizeof(probe))) {
            s_address = 0U;
            return false;
        }
    }
    if (!write_commands(init, sizeof(init))) {
        return false;
    }
    Oled_Clear();
    return Oled_Refresh();
}

void Oled_Clear(void)
{
    uint16_t index;

    for (index = 0U; index < OLED_BUFFER_SIZE; ++index) {
        s_buffer[index] = 0U;
    }
}

void Oled_ShowString(uint8_t x, uint8_t page, const char *text)
{
    if (text == 0) {
        return;
    }
    while ((*text != '\0') && (x <= OLED_WIDTH - 6U)) {
        draw_char(x, page, *text++);
        x = (uint8_t) (x + 6U);
    }
}

bool Oled_Refresh(void)
{
    static const uint8_t window[] = {
        0x21,0x00,0x7F,0x22,0x00,0x07
    };
    uint8_t packet[OLED_PACKET_MAX];
    uint16_t index = 0U;
    uint16_t remaining;
    
    if ((s_address == 0U) || !write_commands(window, sizeof(window))) {
        return false;
    }
    while (index < OLED_BUFFER_SIZE) {
        /* 1024字节显存不能先转换成uint8_t，否则1024会溢出成0，刷新会死循环。 */
        remaining = (uint16_t) (OLED_BUFFER_SIZE - index);
        uint8_t copied = (remaining > 7U) ? 7U : (uint8_t) remaining;
        uint8_t i;

        packet[0] = 0x40U;
        for (i = 0U; i < copied; ++i) {
            packet[i + 1U] = s_buffer[index + i];
        }
        if (!write_packet(s_address, packet, (uint8_t) (copied + 1U))) {
            return false;
        }
        index = (uint16_t) (index + copied);
    }
    return true;
}

uint8_t Oled_GetAddress(void)
{
    return s_address;
}
