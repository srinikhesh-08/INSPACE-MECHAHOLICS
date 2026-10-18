#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "actuators.h"

static const char *TAG = "act";

#define DUTY_STOP     614     /* 1.5 ms: 13-bit timer, 8192 steps per 20 ms */
#define DUTY_FWD_MAX  819     /* 2.0 ms */
#define DUTY_REV_MAX  410     /* 1.0 ms */
#define DUTY_MIN      205     /* 0.5 ms, never go below */
#define DUTY_MAX      1024    /* 2.5 ms, never go above */

class Servo
{
private:
    ledc_channel_t channel_ = LEDC_CHANNEL_0;
    inline static bool timer_ready_ = false;     /* one LEDC timer shared by both servos */

public:
    esp_err_t config(gpio_num_t gpio, ledc_channel_t channel)
    {
        if (!timer_ready_) {
            ledc_timer_config_t timer_conf = {};
            timer_conf.speed_mode      = LEDC_LOW_SPEED_MODE;
            timer_conf.timer_num       = LEDC_TIMER_0;
            timer_conf.duty_resolution = LEDC_TIMER_13_BIT;
            timer_conf.freq_hz         = 50;
            timer_conf.clk_cfg         = LEDC_AUTO_CLK;
            esp_err_t err = ledc_timer_config(&timer_conf);
            if (err != ESP_OK) return err;
            timer_ready_ = true;
        }
        channel_ = channel;                       /* one channel per servo */
        ledc_channel_config_t chnl_conf = {};
        chnl_conf.gpio_num   = gpio;
        chnl_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        chnl_conf.channel    = channel;
        chnl_conf.timer_sel  = LEDC_TIMER_0;
        chnl_conf.duty       = 0;                 /* no pulses: servo stays still */
        chnl_conf.hpoint     = 0;
        return ledc_channel_config(&chnl_conf);
    }

    /* speed in %: +100 = 2.0 ms (forward), 0 = 1.5 ms (stop), -100 = 1.0 ms (reverse) */
    void setSpeed(float speed)
    {
        uint32_t duty;
        if (speed == 0.0f) {
            duty = DUTY_STOP;
        } else if (speed > 0.0f) {
            duty = DUTY_STOP + (uint32_t)((speed / 100.0f) * (DUTY_FWD_MAX - DUTY_STOP));
        } else {
            /* positive offset, then subtract: a negative float cast to uint32_t is undefined */
            duty = DUTY_STOP - (uint32_t)((-speed / 100.0f) * (DUTY_STOP - DUTY_REV_MAX));
        }
        if (duty < DUTY_MIN) duty = DUTY_MIN;
        if (duty > DUTY_MAX) duty = DUTY_MAX;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
    }

    void idle()                                   /* stop sending pulses */
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
    }
};

enum : uint8_t { MOVE_LOCK = 1, MOVE_UNLOCK = 2 };

struct Actuator {
    const char   *name = nullptr;
    Servo         servo;
    QueueHandle_t queue = nullptr;
    uint32_t      lock_ms = 0;
    uint32_t      unlock_ms = 0;
};

static Actuator s_payload;
static Actuator s_parachute;

/* One task per servo, so both can move at the same time at separation. */
static void actuator_task(void *arg)
{
    Actuator *a = static_cast<Actuator *>(arg);
    uint8_t move;
    while (true) {
        if (xQueueReceive(a->queue, &move, portMAX_DELAY) != pdTRUE) continue;
        const bool lock = (move == MOVE_LOCK);
        const float speed = lock ? SERVO_LOCK_SPEED_PCT : SERVO_UNLOCK_SPEED_PCT;
        const uint32_t ms = lock ? a->lock_ms : a->unlock_ms;
        ESP_LOGI(TAG, "%s servo -> %s (%d%% for %lu ms)",
                 a->name, lock ? "LOCK" : "UNLOCK", (int)speed, (unsigned long)ms);
        a->servo.setSpeed(speed);
        vTaskDelay(pdMS_TO_TICKS(ms));
        a->servo.idle();
        ESP_LOGI(TAG, "%s servo done", a->name);
    }
}

static void start_actuator(Actuator &a, const char *name, gpio_num_t gpio,
                           ledc_channel_t ch, uint32_t lock_ms, uint32_t unlock_ms)
{
    a.name = name;
    a.lock_ms = lock_ms;
    a.unlock_ms = unlock_ms;
    if (a.servo.config(gpio, ch) != ESP_OK) {
        ESP_LOGE(TAG, "%s servo PWM setup failed", name);
    }
    a.queue = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreate(actuator_task, name, 4096, &a, 8, nullptr);
}

static void post(Actuator &a, uint8_t move)
{
    if (a.queue == nullptr || xQueueSend(a.queue, &move, 0) != pdTRUE) {
        ESP_LOGE(TAG, "%s servo: command dropped", a.name ? a.name : "?");
    }
}

extern "C" {

void act_init(void)
{
    static bool done = false;
    if (done) return;
    done = true;
    start_actuator(s_payload,   "payload",   (gpio_num_t)PAYLOAD_SERVO_GPIO,   LEDC_CHANNEL_0,
                   PAYLOAD_LOCK_MS,   PAYLOAD_UNLOCK_MS);
    start_actuator(s_parachute, "parachute", (gpio_num_t)PARACHUTE_SERVO_GPIO, LEDC_CHANNEL_1,
                   PARACHUTE_LOCK_MS, PARACHUTE_UNLOCK_MS);
    ESP_LOGI(TAG, "servos ready, assumed UNLOCKED (0 deg), no movement at boot");
}

void act_payload_lock(void)      { post(s_payload,   MOVE_LOCK);   }
void act_payload_unlock(void)    { post(s_payload,   MOVE_UNLOCK); }
void act_parachute_lock(void)    { post(s_parachute, MOVE_LOCK);   }
void act_parachute_unlock(void)  { post(s_parachute, MOVE_UNLOCK); }

}