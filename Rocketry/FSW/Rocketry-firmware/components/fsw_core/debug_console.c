#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fsw_core.h"
#include "debug_console.h"

static const char *TAG = "console";

static void console_task(void *arg)
{
    ESP_LOGI(TAG, "keys: t=ENTER_TEST  l=ENTER_LAUNCHPAD  p=toggle payload  c=toggle parachute  s=TRIGGER_SEP");
    while (true) {
        int c = getchar();
        if (c == EOF) {                          /* nothing typed: wait a bit */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        switch (c) {
        case 't': fsw_post_command(CMD_ENTER_TEST);      break;
        case 'l': fsw_post_command(CMD_ENTER_LAUNCHPAD); break;
        case 'p': fsw_post_command(CMD_TOGGLE_PAYLOAD);   break;
        case 'c': fsw_post_command(CMD_TOGGLE_PARACHUTE); break;
        case 's': fsw_post_command(CMD_TRIGGER_SEP);     break;
        default:  break;
        }
    }
}

void debug_console_start(void)
{
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
}