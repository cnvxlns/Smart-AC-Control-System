#include "app_dht22.h"

#ifndef APP_HOST_TEST

#include <string.h>

#include "app_board.h"
#include "app_config.h"

typedef enum {
    DHT_STAGE_IDLE = 0,
    DHT_STAGE_ENTER = 1,
    DHT_STAGE_START_PULSE = 2,
    DHT_STAGE_CRITICAL_ENTER = 3,
    DHT_STAGE_RESP_LOW = 4,
    DHT_STAGE_RESP_HIGH = 5,
    DHT_STAGE_DATA_DONE = 6,
    DHT_STAGE_CRITICAL_EXIT = 7,
    DHT_STAGE_DONE = 8
} DhtStage;

typedef enum {
    DHT_ERR_NONE = 0,
    DHT_ERR_TIMER_NOT_READY = 1,
    DHT_ERR_RESPONSE_LOW_TIMEOUT = 2,
    DHT_ERR_RESPONSE_HIGH_TIMEOUT = 3,
    DHT_ERR_DATA_LOW_TIMEOUT = 4,
    DHT_ERR_DATA_HIGH_TIMEOUT = 5,
    DHT_ERR_CHECKSUM = 6
} DhtErrorCode;

static volatile bool DhtValid;
static volatile int16_t DhtTemperatureTenths;
static volatile int16_t DhtHumidityTenths;
static volatile uint32_t DhtFailCount;
static volatile uint32_t DhtLastError;
static volatile uint16_t DhtLastRespLowUs;
static volatile uint16_t DhtLastRespHighUs;
static volatile uint16_t DhtLastBitLowUs;
static volatile uint16_t DhtLastBitHighUs;
static volatile uint8_t DhtLastByte0;
static volatile uint8_t DhtLastByte1;
static volatile uint8_t DhtLastByte2;
static volatile uint8_t DhtLastByte3;
static volatile uint8_t DhtLastByte4;
static volatile uint8_t DhtStageValue;

static int16_t AppFloatToTenths(float value)
{
    float scaled = value * 10.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)scaled;
}

static bool DhtMeasurePulse(GPIO_PinState level,
                            uint16_t *duration_us,
                            uint32_t deadline_start_us)
{
    uint32_t pulse_start_us;
    uint32_t guard = 0U;

    while (Board_DhtRead() != level) {
        if (Board_TimerElapsedUs(deadline_start_us) >= APP_DHT_READ_TIMEOUT_US ||
            ++guard >= APP_DHT_LOOP_GUARD) {
            return false;
        }
    }
    pulse_start_us = Board_UsTimerCounter();
    guard = 0U;
    while (Board_DhtRead() == level) {
        if (Board_TimerElapsedUs(deadline_start_us) >= APP_DHT_READ_TIMEOUT_US ||
            Board_TimerElapsedUs(pulse_start_us) >= APP_DHT_PULSE_TIMEOUT_US ||
            ++guard >= APP_DHT_LOOP_GUARD) {
            return false;
        }
    }
    *duration_us = (uint16_t)Board_TimerElapsedUs(pulse_start_us);
    return true;
}

bool DhtRead(float *temperature_c, float *humidity_pct)
{
    uint8_t data[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t byte_index;
    uint8_t bit_index;
    uint16_t low_us = 0U;
    uint16_t high_us = 0U;
    uint16_t resp_low_us = 0U;
    uint16_t resp_high_us = 0U;
    uint16_t bit_low_us = 0U;
    uint16_t bit_high_us = 0U;
    uint16_t raw_humidity;
    uint16_t raw_temperature;
    bool read_ok = true;
    bool checksum_ok = false;
    bool dht_result = false;
    uint32_t deadline_start_us;
    uint32_t primask;
    uint8_t last_error = (uint8_t)DHT_ERR_NONE;

    if (!Board_IsUsTimerReady()) {
        DhtLastError = (uint32_t)DHT_ERR_TIMER_NOT_READY;
        return false;
    }

    DhtStageValue = (uint8_t)DHT_STAGE_ENTER;
    Board_DhtSetInput();
    App_DelayMs(2U);
    Board_DhtSetOutput();
    Board_DhtWrite(GPIO_PIN_RESET);
    Board_DelayUs(APP_DHT22_START_US);
    DhtStageValue = (uint8_t)DHT_STAGE_START_PULSE;

    primask = __get_PRIMASK();
    DhtStageValue = (uint8_t)DHT_STAGE_CRITICAL_ENTER;
#if APP_DHT_DISABLE_IRQ
    __disable_irq();
#endif

    Board_DhtWrite(GPIO_PIN_SET);
    Board_DhtSetInput();
    deadline_start_us = Board_UsTimerCounter();
    Board_DelayUs(APP_DHT22_PULL_US);

    do {
        if (!DhtMeasurePulse(GPIO_PIN_RESET, &resp_low_us,
                             deadline_start_us)) {
            last_error = (uint8_t)DHT_ERR_RESPONSE_LOW_TIMEOUT;
            read_ok = false;
            break;
        }
        DhtStageValue = (uint8_t)DHT_STAGE_RESP_LOW;
        if (!DhtMeasurePulse(GPIO_PIN_SET, &resp_high_us,
                             deadline_start_us)) {
            last_error = (uint8_t)DHT_ERR_RESPONSE_HIGH_TIMEOUT;
            read_ok = false;
            break;
        }
        DhtStageValue = (uint8_t)DHT_STAGE_RESP_HIGH;
        for (byte_index = 0U; byte_index < 5U; ++byte_index) {
            for (bit_index = 0U; bit_index < 8U; ++bit_index) {
                if (!DhtMeasurePulse(GPIO_PIN_RESET, &low_us,
                                     deadline_start_us)) {
                    last_error = (uint8_t)DHT_ERR_DATA_LOW_TIMEOUT;
                    read_ok = false;
                    break;
                }
                if (!DhtMeasurePulse(GPIO_PIN_SET, &high_us,
                                     deadline_start_us)) {
                    last_error = (uint8_t)DHT_ERR_DATA_HIGH_TIMEOUT;
                    read_ok = false;
                    break;
                }
                bit_low_us = low_us;
                bit_high_us = high_us;
                data[byte_index] <<= 1U;
                if (high_us > APP_DHT_BIT_ONE_THRESHOLD_US) {
                    data[byte_index] |= 1U;
                }
            }
            if (!read_ok) {
                break;
            }
        }
        if (read_ok) {
            DhtStageValue = (uint8_t)DHT_STAGE_DATA_DONE;
        }
    } while (false);

#if APP_DHT_DISABLE_IRQ
    if (primask == 0U) {
        __enable_irq();
    }
#else
    (void)primask;
#endif
    DhtStageValue = (uint8_t)DHT_STAGE_CRITICAL_EXIT;

    if (read_ok) {
        checksum_ok =
            ((uint8_t)(data[0] + data[1] + data[2] + data[3]) == data[4]);
        if (!checksum_ok) {
            last_error = (uint8_t)DHT_ERR_CHECKSUM;
        }
    }

    if (read_ok && checksum_ok) {
        raw_humidity = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
        raw_temperature = (uint16_t)(((uint16_t)data[2] << 8U) | data[3]);
        *humidity_pct = (float)raw_humidity * 0.1f;
        if ((raw_temperature & 0x8000U) != 0U) {
            raw_temperature &= 0x7FFFU;
            *temperature_c = -((float)raw_temperature * 0.1f);
        } else {
            *temperature_c = (float)raw_temperature * 0.1f;
        }
        dht_result = true;
        last_error = (uint8_t)DHT_ERR_NONE;
    }

    DhtLastRespLowUs = resp_low_us;
    DhtLastRespHighUs = resp_high_us;
    DhtLastBitLowUs = bit_low_us;
    DhtLastBitHighUs = bit_high_us;
    DhtLastByte0 = data[0];
    DhtLastByte1 = data[1];
    DhtLastByte2 = data[2];
    DhtLastByte3 = data[3];
    DhtLastByte4 = data[4];
    DhtLastError = (uint32_t)last_error;
    if (dht_result) {
        DhtStageValue = (uint8_t)DHT_STAGE_DONE;
    }
    return dht_result;
}

void Dht_RecordSample(bool valid, float temperature_c, float humidity_pct)
{
    DhtValid = valid;
    if (valid) {
        DhtTemperatureTenths = AppFloatToTenths(temperature_c);
        DhtHumidityTenths = AppFloatToTenths(humidity_pct);
    } else {
        ++DhtFailCount;
    }
}

void Dht_GetDiagnostics(DhtDiagnostics *out)
{
    memset(out, 0, sizeof(*out));
    out->valid = DhtValid;
    out->temperature_tenths = DhtTemperatureTenths;
    out->humidity_tenths = DhtHumidityTenths;
    out->fail_count = DhtFailCount;
    out->last_error = DhtLastError;
    out->resp_low_us = DhtLastRespLowUs;
    out->resp_high_us = DhtLastRespHighUs;
    out->bit_low_us = DhtLastBitLowUs;
    out->bit_high_us = DhtLastBitHighUs;
    out->raw[0] = DhtLastByte0;
    out->raw[1] = DhtLastByte1;
    out->raw[2] = DhtLastByte2;
    out->raw[3] = DhtLastByte3;
    out->raw[4] = DhtLastByte4;
    out->stage = DhtStageValue;
}

uint8_t Dht_GetStage(void)
{
    return DhtStageValue;
}

#endif /* APP_HOST_TEST */
