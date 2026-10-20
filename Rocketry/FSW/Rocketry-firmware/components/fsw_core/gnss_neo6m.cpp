#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared_state.h"

#define GNSS_UART     UART_NUM_2
#define GNSS_BAUD     115200    
#define GNSS_PIN_TX   17        
#define GNSS_PIN_RX   16         

static const char* tag = "gnss";

struct gga_data
{
    double lat = 0.0;
    double lon = 0.0;
    double altitude = 0.0;
    int satellites = 0;
    uint32_t utc_hhmmss = 0;
};

class uart_manager
{
public:
    void initialise()
    {
        uart_config_t uart_config = {};
        uart_config.baud_rate = GNSS_BAUD;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.rx_flow_ctrl_thresh = 122;
        uart_config.source_clk = UART_SCLK_DEFAULT;
        uart_param_config(GNSS_UART, &uart_config);

        uart_set_pin(GNSS_UART, GNSS_PIN_TX, GNSS_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(GNSS_UART, 2048, 0, 0, NULL, 0);    /* no event queue: nobody reads it */
    }

    bool read_sentence(char gps_data_buffer[], int size)
    {
        uint8_t gps_data_byte = 0;
        int index = 0;

        int waited = 0;                                          /* wait for '$', give up after 1 s of silence */
        while (true) {
            if (uart_read_bytes(GNSS_UART, &gps_data_byte, 1, pdMS_TO_TICKS(10)) == 1) {
                if (static_cast<char>(gps_data_byte) == '$') break;
            } else if (++waited >= 100) {
                return false;
            }
        }
        gps_data_buffer[index++] = '$';

        while (true) {
            if (uart_read_bytes(GNSS_UART, &gps_data_byte, 1, pdMS_TO_TICKS(10)) != 1) {
                return false;                                    /* sentence cut off */
            }
            if (static_cast<char>(gps_data_byte) != '\n') {
                gps_data_buffer[index++] = static_cast<char>(gps_data_byte);
                if (index > size - 1) return false;              /* too long */
            } else {
                if (index > 0 && gps_data_buffer[index - 1] == '\r') index--;
                gps_data_buffer[index] = '\0';
                return true;
            }
        }
    }
};

static bool nmea_to_degrees(const char* value, const char* hemisphere, double &out)
{
    if (value[0] == '\0' || hemisphere[0] == '\0') return false;   /* empty field */

    double raw = strtod(value, nullptr);
    int degrees = static_cast<int>(raw / 100.0);
    double minutes = raw - degrees * 100.0;

    out = degrees + minutes / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') out = -out;
    return true;
}

static bool parse_sentence(char sentence[], char* field_location[], gga_data &out)
{
    /* turn every comma into '\0' so each field becomes its own string */
    char* p = sentence;
    int fields = 0;
    field_location[fields++] = p;

    for (; *p != '\0'; p++) {
        if (*p == ',') {
            if (fields >= 31) return false;                   /* array holds 32 entries */
            field_location[fields++] = p + 1;
            *p = '\0';
        } else if (*p == '*') {
            *p = '\0';
            break;
        }
    }

    if (strcmp(field_location[0], "$GPGGA") != 0 && strcmp(field_location[0], "$GNGGA") != 0) {
        return false;                                         /* some other sentence type */
    }
    if (fields < 10) return false;                            /* damaged sentence */

    if (*field_location[6] == '0') {
        ESP_LOGD(tag, "No fix yet");
        return false;
    }

    double lat, lon;
    if (!nmea_to_degrees(field_location[2], field_location[3], lat)) return false;
    if (!nmea_to_degrees(field_location[4], field_location[5], lon)) return false;
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;

    out.lat = lat;                                            /* write only after all checks pass */
    out.lon = lon;
    out.satellites = atoi(field_location[7]);
    out.altitude = strtod(field_location[9], nullptr);
    out.utc_hhmmss = (uint32_t)strtod(field_location[1], nullptr);   /* hhmmss.ss -> hhmmss */
    return true;
}

static void gnss_task(void *arg)
{
    uart_manager gps;
    gps.initialise();

    char line[128];
    char* f[32];
    gga_data gga;
    gnss_data_t g = {};
    uint32_t last_good_ms = 0;

    while (true) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (gps.read_sentence(line, sizeof(line)) && parse_sentence(line, f, gga)) {
            g.t_ms       = now;
            g.utc_hhmmss = gga.utc_hhmmss;
            g.lat        = (float)gga.lat;
            g.lon        = (float)gga.lon;
            g.alt_msl    = (float)gga.altitude;
            g.sats       = (uint8_t)gga.satellites;
            g.fix        = true;
            g.valid      = true;
            last_good_ms = now;
            shared_write_gnss(&g);
        } else if (g.valid && (now - last_good_ms) > 3000) {
            g.fix   = false;                                  /* fix lost or GPS silent */
            g.valid = false;
            shared_write_gnss(&g);
        }
    }
}

extern "C" void gnss_start(void)
{
    xTaskCreate(gnss_task, "gnss", 4096, NULL, 6, NULL);
}