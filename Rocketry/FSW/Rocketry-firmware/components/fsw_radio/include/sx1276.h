#pragma once
#include <stdint.h>
#include "esp_err.h"

/* SX1276 pins: ASSUMED, change to your wiring */
#define LORA_PIN_SCK    27
#define LORA_PIN_MISO   19
#define LORA_PIN_MOSI   23
#define LORA_PIN_CS     5
#define LORA_PIN_RST    14

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t lora_init(void);                                   /* SPI + reset + chip check */
esp_err_t lora_configure(uint32_t freq_hz, uint8_t sf, uint8_t sync_word);
void      lora_set_power_dbm(int dbm);                       /* 2..17 dBm */
esp_err_t lora_send(const uint8_t *data, uint8_t len);       /* waits until sent */
void      lora_start_rx(void);                               /* listen (no transmission) */
int       lora_poll_rx(uint8_t *buf, uint8_t max_len);       /* bytes received, 0 = nothing */
void      lora_sleep(void);
int   lora_last_rssi(void);      /* dBm of the last received packet */
float lora_last_snr(void);       /* dB */
#ifdef __cplusplus
}
#endif