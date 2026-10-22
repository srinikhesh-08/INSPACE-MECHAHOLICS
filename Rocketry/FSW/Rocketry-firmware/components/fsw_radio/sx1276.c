#include "sx1276.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LORA_SPI_HOST    SPI3_HOST          /* the SD card uses SPI2_HOST */

static const char *TAG = "lora";
static spi_device_handle_t s_dev = NULL;

#define REG_FIFO            0x00
#define REG_OP_MODE         0x01
#define REG_FRF_MSB         0x06
#define REG_PA_CONFIG       0x09
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_FIFO_RX_CURRENT 0x10
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_PREAMBLE_MSB    0x20
#define REG_PAYLOAD_LENGTH  0x22
#define REG_MODEM_CONFIG3   0x26
#define REG_SYNC_WORD       0x39
#define REG_VERSION         0x42
#define REG_PKT_SNR         0x19
#define REG_PKT_RSSI        0x1A

#define MODE_LORA     0x80
#define MODE_SLEEP    0x00
#define MODE_STDBY    0x01
#define MODE_TX       0x03
#define MODE_RX_CONT  0x05
#define IRQ_TX_DONE   0x08
#define IRQ_CRC_ERR   0x20
#define IRQ_RX_DONE   0x40

static uint8_t rd(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(s_dev, &t);
    return rx[1];
}

static void wr(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_polling_transmit(s_dev, &t);
}

static void fifo_write(const uint8_t *data, uint8_t len)
{
    static uint8_t tx[256];
    tx[0] = REG_FIFO | 0x80;
    memcpy(&tx[1], data, len);
    spi_transaction_t t = { .length = (size_t)(len + 1) * 8, .tx_buffer = tx };
    spi_device_polling_transmit(s_dev, &t);
}

static void fifo_read(uint8_t *out, uint8_t len)
{
    static uint8_t tx[256], rx[256];
    memset(tx, 0, (size_t)len + 1);
    tx[0] = REG_FIFO & 0x7F;
    spi_transaction_t t = { .length = (size_t)(len + 1) * 8, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(s_dev, &t);
    memcpy(out, &rx[1], len);
}

static int   s_rssi = 0;
static float s_snr  = 0.0f;
esp_err_t lora_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = LORA_PIN_MOSI, .miso_io_num = LORA_PIN_MISO, .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 300,
    };
    esp_err_t e = spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 5 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = LORA_PIN_CS,
        .queue_size = 1,
    };
    e = spi_bus_add_device(LORA_SPI_HOST, &dev, &s_dev);
    if (e != ESP_OK) return e;

    gpio_reset_pin(LORA_PIN_RST);                 /* hardware reset pulse */
    gpio_set_direction(LORA_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t v = rd(REG_VERSION);
    if (v != 0x12) {
        ESP_LOGE(TAG, "SX1276 not found (version 0x%02X, expected 0x12): check wiring", v);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "SX1276 found");
    return ESP_OK;
}

esp_err_t lora_configure(uint32_t freq_hz, uint8_t sf, uint8_t sync_word)
{
    if (sf < 7 || sf > 11) return ESP_ERR_INVALID_ARG;

    wr(REG_OP_MODE, MODE_LORA | MODE_SLEEP);          /* LoRa mode can only be selected from sleep */
    vTaskDelay(pdMS_TO_TICKS(10));
    wr(REG_OP_MODE, MODE_LORA | MODE_STDBY);

    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;   /* frequency in 61 Hz steps */
    wr(REG_FRF_MSB,     (uint8_t)(frf >> 16));
    wr(REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
    wr(REG_FRF_MSB + 2, (uint8_t)(frf));

    wr(REG_MODEM_CONFIG1, 0x72);                      /* 125 kHz, coding rate 4/5, explicit header */
    wr(REG_MODEM_CONFIG2, (uint8_t)((sf << 4) | 0x04));   /* spreading factor, CRC on */
    uint8_t mc3 = 0x04;                               /* automatic gain control */
    if (sf >= 11) mc3 |= 0x08;                        /* low data rate optimise */
    wr(REG_MODEM_CONFIG3, mc3);

    wr(REG_PREAMBLE_MSB, 0x00);
    wr(REG_PREAMBLE_MSB + 1, 0x08);                   /* 8-symbol preamble */
    wr(REG_SYNC_WORD, sync_word);
    wr(REG_LNA, rd(REG_LNA) | 0x03);                  /* LNA boost */
    wr(REG_FIFO_TX_BASE, 0x00);
    wr(REG_FIFO_RX_BASE, 0x00);

    ESP_LOGI(TAG, "configured: %lu Hz, SF%u, BW 125 kHz, CR 4/5, sync 0x%02X",
             (unsigned long)freq_hz, sf, sync_word);
    return ESP_OK;
}

void lora_set_power_dbm(int dbm)
{
    if (dbm < 2)  dbm = 2;
    if (dbm > 17) dbm = 17;
    wr(REG_PA_CONFIG, (uint8_t)(0x80 | (dbm - 2)));   /* PA_BOOST pin: power = 2 + setting */
}

esp_err_t lora_send(const uint8_t *data, uint8_t len)
{
    wr(REG_OP_MODE, MODE_LORA | MODE_STDBY);
    wr(REG_FIFO_ADDR_PTR, 0x00);
    fifo_write(data, len);
    wr(REG_PAYLOAD_LENGTH, len);
    wr(REG_IRQ_FLAGS, 0xFF);
    wr(REG_OP_MODE, MODE_LORA | MODE_TX);

    for (int i = 0; i < 100; i++) {                   /* a packet takes about 0.13 s at SF7 */
        if (rd(REG_IRQ_FLAGS) & IRQ_TX_DONE) {
            wr(REG_IRQ_FLAGS, 0xFF);
            wr(REG_OP_MODE, MODE_LORA | MODE_STDBY);
            return ESP_OK;
        }
        vTaskDelay(1);
    }
    wr(REG_OP_MODE, MODE_LORA | MODE_STDBY);          /* timed out: make sure we stop transmitting */
    return ESP_ERR_TIMEOUT;
}

void lora_start_rx(void)
{
    wr(REG_IRQ_FLAGS, 0xFF);
    wr(REG_OP_MODE, MODE_LORA | MODE_RX_CONT);
}

int lora_poll_rx(uint8_t *buf, uint8_t max_len)
{
    uint8_t irq = rd(REG_IRQ_FLAGS);
    if (!(irq & IRQ_RX_DONE)) return 0;
    wr(REG_IRQ_FLAGS, 0xFF);
    if (irq & IRQ_CRC_ERR) return 0;                  /* damaged packet: ignore it */

    s_snr = (int8_t)rd(REG_PKT_SNR) / 4.0f;
    int rssi = -157 + rd(REG_PKT_RSSI);           /* high-frequency port (above 525 MHz) */
    if (s_snr < 0) rssi += (int)s_snr;            /* correction for packets below the noise floor */
    s_rssi = rssi;

    uint8_t len = rd(REG_RX_NB_BYTES);
    if (len == 0 || len > max_len) return 0;
    wr(REG_FIFO_ADDR_PTR, rd(REG_FIFO_RX_CURRENT));
    fifo_read(buf, len);
    return len;
}

void lora_sleep(void)
{
    wr(REG_OP_MODE, MODE_LORA | MODE_SLEEP);
}

int   lora_last_rssi(void) { return s_rssi; }
float lora_last_snr(void)  { return s_snr; }