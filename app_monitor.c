#include "app_monitor.h"

#ifndef APP_HOST_TEST

#include "app_board.h"
#include "app_dht22.h"
#include "app_tm1637.h"

static OS_MUTEX *MonitorUartMutex;

void Monitor_Init(OS_MUTEX *uart_mutex)
{
    MonitorUartMutex = uart_mutex;
}

static uint16_t AppStrLen(const char *text)
{
    uint16_t length = 0U;

    while (text[length] != '\0') {
        ++length;
    }
    return length;
}

void Monitor_WriteBytesUnlocked(const char *buffer, uint16_t length)
{
    Board_UartWriteBytes(buffer, length);
}

void Monitor_WriteStringUnlocked(const char *text)
{
    Monitor_WriteBytesUnlocked(text, AppStrLen(text));
}

void Monitor_WriteUIntUnlocked(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        Monitor_WriteBytesUnlocked("0", 1U);
        return;
    }

    while (value != 0U && count < sizeof(digits)) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) {
        --count;
        Monitor_WriteBytesUnlocked(&digits[count], 1U);
    }
}

static void UartWriteTenthsUnlocked(int16_t value)
{
    int32_t signed_value = (int32_t)value;
    uint32_t whole;
    uint32_t fraction;

    if (signed_value < 0) {
        Monitor_WriteBytesUnlocked("-", 1U);
        signed_value = -signed_value;
    }
    whole = (uint32_t)(signed_value / 10);
    fraction = (uint32_t)(signed_value % 10);
    Monitor_WriteUIntUnlocked(whole);
    Monitor_WriteBytesUnlocked(".", 1U);
    Monitor_WriteUIntUnlocked(fraction);
}

void Monitor_WriteHexUnlocked(uint32_t value)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    int8_t shift;

    Monitor_WriteBytesUnlocked("0x", 2U);
    for (shift = 28; shift >= 0; shift -= 4) {
        char nibble = hex_digits[(value >> (uint8_t)shift) & 0xFU];
        Monitor_WriteBytesUnlocked(&nibble, 1U);
    }
}

void Monitor_Log(const char *text)
{
    OS_ERR err;

    OSMutexPend(MonitorUartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        Monitor_WriteStringUnlocked(text);
        OSMutexPost(MonitorUartMutex, OS_OPT_POST_NONE, &err);
    }
}

void Monitor_LogTimerProbe(uint32_t ticks)
{
    OS_ERR err;

    OSMutexPend(MonitorUartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        Monitor_WriteStringUnlocked("TIM2 ticks/probe=");
        Monitor_WriteUIntUnlocked(ticks);
        Monitor_WriteStringUnlocked(" (expect ~");
        Monitor_WriteUIntUnlocked((uint32_t)APP_US_TIMER_PROBE_MS * 1000U);
        Monitor_WriteStringUnlocked(" for 1MHz)\r\n");
        OSMutexPost(MonitorUartMutex, OS_OPT_POST_NONE, &err);
    }
}

void Monitor_LogTaskCreateFailed(const CPU_CHAR *name, OS_ERR task_err)
{
    OS_ERR err;

    OSMutexPend(MonitorUartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        Monitor_WriteStringUnlocked("Task create failed: ");
        Monitor_WriteStringUnlocked((const char *)name);
        Monitor_WriteStringUnlocked(" err=");
        Monitor_WriteUIntUnlocked((uint32_t)task_err);
        Monitor_WriteStringUnlocked("\r\n");
        OSMutexPost(MonitorUartMutex, OS_OPT_POST_NONE, &err);
    }
}

void Monitor_LogInputs(const AppRuntimeDiagnostics *runtime)
{
    OS_ERR err;
    bool up_pressed = Board_PinIsPressed(APP_UP_PORT, APP_UP_PIN);
    bool down_pressed = Board_PinIsPressed(APP_DOWN_PORT, APP_DOWN_PIN);
    bool b1_pressed = Board_B1IsPressed();
    bool reed_open = Board_ReedIsOpen();
    bool us_timer_ready = Board_IsUsTimerReady();
    uint8_t tm1637_ack = Tm1637_GetAckCount();
    DhtDiagnostics dht;

    Dht_GetDiagnostics(&dht);

    OSMutexPend(MonitorUartMutex, 0U, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE) {
        Monitor_WriteStringUnlocked("INPUT button_up=");
        Monitor_WriteBytesUnlocked(up_pressed ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" button_down=");
        Monitor_WriteBytesUnlocked(down_pressed ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" button_b1=");
        Monitor_WriteBytesUnlocked(b1_pressed ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" reed_open=");
        Monitor_WriteBytesUnlocked(reed_open ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" dht22=");
        Monitor_WriteBytesUnlocked(dht.valid ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" temp_c=");
        if (dht.valid) {
            UartWriteTenthsUnlocked(dht.temperature_tenths);
        } else {
            Monitor_WriteStringUnlocked("--.-");
        }
        Monitor_WriteStringUnlocked(" humidity_pct=");
        if (dht.valid) {
            UartWriteTenthsUnlocked(dht.humidity_tenths);
        } else {
            Monitor_WriteStringUnlocked("--.-");
        }
        Monitor_WriteStringUnlocked(" dht_fail_count=");
        Monitor_WriteUIntUnlocked(dht.fail_count);
        Monitor_WriteStringUnlocked(" dht_err=");
        Monitor_WriteUIntUnlocked(dht.last_error);
        Monitor_WriteStringUnlocked(" dht_stage=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.stage);
        Monitor_WriteStringUnlocked(" resp_low_us=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.resp_low_us);
        Monitor_WriteStringUnlocked(" resp_high_us=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.resp_high_us);
        Monitor_WriteStringUnlocked(" bit_low_us=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.bit_low_us);
        Monitor_WriteStringUnlocked(" bit_high_us=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.bit_high_us);
        Monitor_WriteStringUnlocked(" us_timer_ready=");
        Monitor_WriteBytesUnlocked(us_timer_ready ? "1" : "0", 1U);
        Monitor_WriteStringUnlocked(" us_ticks_100ms=");
        Monitor_WriteUIntUnlocked(runtime->us_ticks_per_probe);
        Monitor_WriteStringUnlocked(" sensor_stk_free=");
        Monitor_WriteUIntUnlocked((uint32_t)runtime->sensor_stk_free);
        Monitor_WriteStringUnlocked(" tm1637_ack=");
        Monitor_WriteUIntUnlocked((uint32_t)tm1637_ack);
        Monitor_WriteStringUnlocked(" raw=");
        Monitor_WriteUIntUnlocked((uint32_t)dht.raw[0]);
        Monitor_WriteBytesUnlocked(",", 1U);
        Monitor_WriteUIntUnlocked((uint32_t)dht.raw[1]);
        Monitor_WriteBytesUnlocked(",", 1U);
        Monitor_WriteUIntUnlocked((uint32_t)dht.raw[2]);
        Monitor_WriteBytesUnlocked(",", 1U);
        Monitor_WriteUIntUnlocked((uint32_t)dht.raw[3]);
        Monitor_WriteBytesUnlocked(",", 1U);
        Monitor_WriteUIntUnlocked((uint32_t)dht.raw[4]);
        Monitor_WriteStringUnlocked("\r\n");
        OSMutexPost(MonitorUartMutex, OS_OPT_POST_NONE, &err);
    }
}

#endif /* APP_HOST_TEST */
