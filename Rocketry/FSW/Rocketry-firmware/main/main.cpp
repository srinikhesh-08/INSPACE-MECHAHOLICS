#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_state.h"

static void baro_writer(void *arg)
{
    baro_data_t b = {};
    b.valid = true;
    b.pressure_pa = 101325.0f;
    while (true) {
        b.t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        shared_write_baro(&b);
        b.pressure_pa -= 1.0f;          /* fake: pressure slowly falls */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void baro_reader(void *arg)
{
    baro_data_t b;
    while (true) {
        shared_read_baro(&b);
        printf("reader: t=%lu ms  pressure=%.1f Pa  valid=%d\n",
               (unsigned long)b.t_ms, b.pressure_pa, b.valid);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    shared_init();
    xTaskCreate(baro_writer, "baro_w", 4096, nullptr, 6, nullptr);
    xTaskCreate(baro_reader, "baro_r", 4096, nullptr, 4, nullptr);
}