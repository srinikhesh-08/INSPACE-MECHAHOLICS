#include "driver/ledc.h"
#include "esp_log.h"
#include "config.h"
#include "actuators.h"

static const char *TAG = "act";

#define SERVO_MODE        LEDC_LOW_SPEED_MODE
#define SERVO_TIMER       LEDC_TIMER_0
#define SERVO_RES_BITS    14
#define SERVO_CH_PAYLOAD  LEDC_CHANNEL_0
#define SERVO_CH_CHUTE    LEDC_CHANNEL_1

/* angle (deg) -> pulse width (us) -> LEDC duty counts */
static uint32_t angle_to_duty(float deg)
{
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float pulse_us  = SERVO_PULSE_MIN_US + (deg / 180.0f) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    float period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;
    return (uint32_t)(pulse_us / period_us * (float)(1u << SERVO_RES_BITS) + 0.5f);
}

static void servo_move(ledc_channel_t ch, float deg)
{
    esp_err_t e = ledc_set_duty(SERVO_MODE, ch, angle_to_duty(deg));
    if (e == ESP_OK) {
        e = ledc_update_duty(SERVO_MODE, ch);
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "servo PWM error: %s", esp_err_to_name(e));
    }
}

void act_init(bool payload_locked, bool parachute_locked)
{
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

     /* The initial duty is the position the FSW wants (unlocked on a clean boot, the saved position
       after an in-flight reset), so a servo never moves just because PWM starts. */
    ledc_channel_config_t ch = {
        .gpio_num   = SERVO_PAYLOAD_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CH_PAYLOAD,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = angle_to_duty(payload_locked ? SERVO_LOCKED_DEG : SERVO_UNLOCKED_DEG),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ch.gpio_num = SERVO_PARACHUTE_GPIO;
    ch.channel  = SERVO_CH_CHUTE;
    ch.duty     = angle_to_duty(parachute_locked ? SERVO_LOCKED_DEG : SERVO_UNLOCKED_DEG);
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ESP_LOGI(TAG, "servo PWM ready: payload GPIO%d %s, parachute GPIO%d %s",
             SERVO_PAYLOAD_GPIO, payload_locked ? "LOCKED" : "UNLOCKED",
             SERVO_PARACHUTE_GPIO, parachute_locked ? "LOCKED" : "UNLOCKED");
}

void act_payload_lock(void)
{
    servo_move(SERVO_CH_PAYLOAD, SERVO_LOCKED_DEG);
    ESP_LOGI(TAG, "payload cabinet servo   -> LOCK   (%.0f deg)", SERVO_LOCKED_DEG);
}

void act_payload_unlock(void)
{
    servo_move(SERVO_CH_PAYLOAD, SERVO_UNLOCKED_DEG);
    ESP_LOGI(TAG, "payload cabinet servo   -> UNLOCK (%.0f deg)", SERVO_UNLOCKED_DEG);
}

void act_parachute_lock(void)
{
    servo_move(SERVO_CH_CHUTE, SERVO_LOCKED_DEG);
    ESP_LOGI(TAG, "parachute cabinet servo -> LOCK   (%.0f deg)", SERVO_LOCKED_DEG);
}

void act_parachute_unlock(void)
{
    servo_move(SERVO_CH_CHUTE, SERVO_UNLOCKED_DEG);
    ESP_LOGI(TAG, "parachute cabinet servo -> UNLOCK (%.0f deg)", SERVO_UNLOCKED_DEG);
}