#include "esp_log.h"
#include "actuators.h"

static const char *TAG = "act";

void act_payload_lock(void)      { ESP_LOGI(TAG, "payload cabinet servo   -> LOCK   (180 deg)"); }
void act_payload_unlock(void)    { ESP_LOGI(TAG, "payload cabinet servo   -> UNLOCK (0 deg)"); }
void act_parachute_lock(void)    { ESP_LOGI(TAG, "parachute cabinet servo -> LOCK   (180 deg)"); }
void act_parachute_unlock(void)  { ESP_LOGI(TAG, "parachute cabinet servo -> UNLOCK (0 deg)"); }