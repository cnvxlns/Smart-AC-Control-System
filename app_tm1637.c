#include "app_tm1637.h"

#ifndef APP_HOST_TEST

#include <stdbool.h>

#include "app_board.h"
#include "app_config.h"

static volatile uint8_t Tm1637AckCount;

static void Tm1637Delay(void)
{
    Board_DelayUs(APP_TM1637_DELAY_US);
}

static void Tm1637Start(void)
{
    Board_Tm1637WriteDio(true);
    Board_Tm1637WriteClk(true);
    Tm1637Delay();
    Board_Tm1637WriteDio(false);
    Tm1637Delay();
    Board_Tm1637WriteClk(false);
}

static void Tm1637Stop(void)
{
    Board_Tm1637WriteClk(false);
    Board_Tm1637WriteDio(false);
    Tm1637Delay();
    Board_Tm1637WriteClk(true);
    Tm1637Delay();
    Board_Tm1637WriteDio(true);
    Tm1637Delay();
}

static bool Tm1637WriteByte(uint8_t value)
{
    uint8_t bit_index;
    bool ack;

    for (bit_index = 0U; bit_index < 8U; ++bit_index) {
        Board_Tm1637WriteClk(false);
        Board_Tm1637WriteDio((value & 0x01U) != 0U);
        Tm1637Delay();
        Board_Tm1637WriteClk(true);
        Tm1637Delay();
        value >>= 1U;
    }

    Board_Tm1637WriteClk(false);
    Board_Tm1637WriteDio(true);
    Tm1637Delay();
    Board_Tm1637WriteClk(true);
    Tm1637Delay();
    ack = Board_Tm1637ReadDio() == GPIO_PIN_RESET;
    Board_Tm1637WriteClk(false);
    return ack;
}

void Tm1637_SetSegments(const uint8_t segments[4])
{
    uint8_t index;
    uint8_t ack_count = 0U;
    uint8_t display_cmd =
        (uint8_t)(0x88U | (APP_TM1637_BRIGHTNESS & 0x07U));

    Tm1637Start();
    if (Tm1637WriteByte(0x40U)) { ++ack_count; }
    Tm1637Stop();

    Tm1637Start();
    if (Tm1637WriteByte(0xC0U)) { ++ack_count; }
    for (index = 0U; index < 4U; ++index) {
        if (Tm1637WriteByte(segments[index])) { ++ack_count; }
    }
    Tm1637Stop();

    Tm1637Start();
    if (Tm1637WriteByte(display_cmd)) { ++ack_count; }
    Tm1637Stop();

    Tm1637AckCount = ack_count;
}

static void Tm1637ClearSegments(uint8_t segments[4])
{
    uint8_t index;

    for (index = 0U; index < 4U; ++index) {
        segments[index] = 0x00U;
    }
}

static uint8_t Tm1637DigitSegment(uint8_t digit)
{
    static const uint8_t digit_segments[10] = {
        0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
        0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
    };

    return (digit < 10U) ? digit_segments[digit] : 0x00U;
}

static uint8_t Tm1637CharSegment(char value)
{
    switch (value) {
    case '0':
    case 'O':
        return 0x3FU;
    case 'E':
        return 0x79U;
    case 'F':
        return 0x71U;
    case 'L':
        return 0x38U;
    case 'P':
        return 0x73U;
    case 'S':
        return 0x6DU;
    case 'A':
        return 0x77U;
    case 'U':
        return 0x3EU;
    case '-':
        return 0x40U;
    case ' ':
        return 0x00U;
    default:
        return 0x00U;
    }
}

static void Tm1637PutTwoDigits(int16_t value, bool zero_pad,
                               uint8_t *hi, uint8_t *lo)
{
    if (value < 0) {
        value = (int16_t)(-value);
        if (value > 9) {
            value = 9;
        }
        *hi = Tm1637CharSegment('-');
        *lo = Tm1637DigitSegment((uint8_t)value);
        return;
    }
    if (value > 99) {
        value = 99;
    }
    if (value >= 10) {
        *hi = Tm1637DigitSegment((uint8_t)(value / 10));
    } else {
        *hi = zero_pad ? Tm1637DigitSegment(0U) : Tm1637CharSegment(' ');
    }
    *lo = Tm1637DigitSegment((uint8_t)(value % 10));
}

static void Tm1637PutTemperature(const DisplayMsg *message,
                                 uint8_t *hi, uint8_t *lo)
{
    if (!message->temperature_valid) {
        *hi = Tm1637CharSegment('-');
        *lo = Tm1637CharSegment('-');
    } else {
        Tm1637PutTwoDigits((int16_t)message->current_temperature_c,
                           false, hi, lo);
    }
}

static void Tm1637RenderChars(char c0, char c1, char c2, char c3,
                              uint8_t segments[4])
{
    segments[0] = Tm1637CharSegment(c0);
    segments[1] = Tm1637CharSegment(c1);
    segments[2] = Tm1637CharSegment(c2);
    segments[3] = Tm1637CharSegment(c3);
}

static void Tm1637RenderTwoTwoNumbers(int16_t left_value, int16_t right_value,
                                      bool separator_dp, uint8_t segments[4])
{
    Tm1637PutTwoDigits(left_value, false, &segments[0], &segments[1]);
    Tm1637PutTwoDigits(right_value, true, &segments[2], &segments[3]);
    if (separator_dp) {
        segments[1] |= 0x80U;
    }
}

static void Tm1637RenderTempAndLevel(const DisplayMsg *message,
                                     char level_prefix, uint8_t level,
                                     uint8_t segments[4])
{
    Tm1637PutTemperature(message, &segments[0], &segments[1]);
    segments[2] = Tm1637CharSegment(level_prefix);
    segments[3] = Tm1637DigitSegment((uint8_t)(level % 10U));
}

static void Tm1637RenderCodeAndSeconds(char c0, char c1, uint8_t seconds,
                                       uint8_t segments[4])
{
    segments[0] = Tm1637CharSegment(c0);
    segments[1] = Tm1637CharSegment(c1);
    Tm1637PutTwoDigits((int16_t)seconds, true, &segments[2], &segments[3]);
}

void Tm1637_BuildSegments(const DisplayMsg *message, uint32_t now_ms,
                          uint8_t segments[4])
{
    static AcState last_display_state = AC_STATE_OFF;
    static uint32_t display_state_since_ms = 0U;
    uint32_t elapsed_ms;
    uint32_t seconds;

    if (message->state != last_display_state) {
        last_display_state = message->state;
        display_state_since_ms = now_ms;
    }
    elapsed_ms = now_ms - display_state_since_ms;

    Tm1637ClearSegments(segments);

    switch (message->state) {
    case AC_STATE_OFF:
        if ((now_ms % 1000U) < 500U) {
            Tm1637RenderChars('O', 'F', 'F', ' ', segments);
        }
        break;
    case AC_STATE_IDLE:
        if (message->temperature_valid) {
            Tm1637RenderTwoTwoNumbers((int16_t)message->current_temperature_c,
                                      (int16_t)message->setpoint_c,
                                      true, segments);
        } else {
            Tm1637RenderChars('-', '-', '-', '-', segments);
        }
        break;
    case AC_STATE_COOLING:
        Tm1637RenderTempAndLevel(message, 'L', message->cooling_level, segments);
        break;
    case AC_STATE_WINDOW_SUSPECT:
        seconds = elapsed_ms / 1000U;
        if (seconds > 99U) {
            seconds = 99U;
        }
        Tm1637RenderCodeAndSeconds('O', 'P', (uint8_t)seconds, segments);
        break;
    case AC_STATE_AC_PAUSED:
        Tm1637RenderChars('P', 'A', 'U', 'S', segments);
        break;
    case AC_STATE_RECOVERY_WAIT:
        {
            uint32_t remaining_ms = (elapsed_ms >= APP_RECOVERY_WAIT_MS)
                                        ? 0U
                                        : (APP_RECOVERY_WAIT_MS - elapsed_ms);
            seconds = (remaining_ms + 999U) / 1000U;
            if (seconds > 99U) {
                seconds = 99U;
            }
            Tm1637RenderCodeAndSeconds('E', 'S', (uint8_t)seconds, segments);
        }
        break;
    case AC_STATE_MANUAL_OVERRIDE:
        Tm1637RenderTempAndLevel(message, 'A', message->cooling_level, segments);
        break;
    default:
        Tm1637RenderChars('-', '-', '-', '-', segments);
        break;
    }
}

void Tm1637_Init(void)
{
    uint8_t segments[4];

    Board_Tm1637WriteClk(true);
    Board_Tm1637WriteDio(true);
    Tm1637ClearSegments(segments);
    Tm1637_SetSegments(segments);
}

uint8_t Tm1637_GetAckCount(void)
{
    return Tm1637AckCount;
}

#endif /* APP_HOST_TEST */
