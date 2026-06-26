#ifndef APP_DHT22_H
#define APP_DHT22_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    int16_t temperature_tenths;
    int16_t humidity_tenths;
    uint32_t fail_count;
    uint32_t last_error;
    uint16_t resp_low_us;
    uint16_t resp_high_us;
    uint16_t bit_low_us;
    uint16_t bit_high_us;
    uint8_t raw[5];
    uint8_t stage;
} DhtDiagnostics;

#ifndef APP_HOST_TEST

bool DhtRead(float *temperature_c, float *humidity_pct);
void Dht_RecordSample(bool valid, float temperature_c, float humidity_pct);
void Dht_GetDiagnostics(DhtDiagnostics *out);
uint8_t Dht_GetStage(void);

#endif /* APP_HOST_TEST */

#endif /* APP_DHT22_H */
