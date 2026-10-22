#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fsw_core.h"
#include "debug_console.h"
#include "config.h"
#include "sim_sensors.h"
static const char *TAG = "console";

static void console_task(void *arg)
{
        ESP_LOGI(TAG, "keys: t=ENTER_TEST  l=ENTER_LAUNCHPAD  p=toggle payload  c=toggle parachute  s=TRIGGER_SEP  o=TX ON  f=TX OFF  h=switch channel");
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
        case 'o': fsw_post_command(CMD_START_TELEMETRY);  break;
        case 'f': fsw_post_command(CMD_STOP_TELEMETRY);   break;
        case 'h': fsw_post_command(CMD_SWITCH_CHANNEL);   break;
    #if USE_SIM_SENSORS
        case 'g':
            if (fsw_get_state() == ST_LAUNCH_PAD) {
                ESP_LOGW(TAG, "SIM LAUNCH commanded");
                sim_request_launch();
            } else {
                ESP_LOGW(TAG, "'g' ignored: FSM is in %s, not LAUNCH_PAD",
                         fsw_state_name(fsw_get_state()));
            }
            break;
    #endif
        default:  break;
        }
    }
}

void debug_console_start(void)
{
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
}