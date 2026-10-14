#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "actuators.h"
#include "fsw_core.h"

static const char *TAG = "fsm";

static const char *const STATE_NAME[ST_COUNT] = {
    "BOOT", "TEST_MODE", "LAUNCH_PAD", "ASCENT", "PAYLOAD_SEP", "DESCENT", "IMPACT"
};

/* Task rates in Hz per state (spec table). Column order must match rate_task_t. */
static const uint16_t RATE_HZ[ST_COUNT][RT_COUNT] = {
    /*                 IMU  BARO GNSS DATA */
    /* BOOT        */ {  10,   5,   1,   5 },
    /* TEST_MODE   */ {  10,   5,   1,   5 },
    /* LAUNCH_PAD  */ {  10,   5,   5,   5 },
    /* ASCENT      */ { 100,  20,   5,  20 },
    /* PAYLOAD_SEP */ {  50,  10,   5,  10 },
    /* DESCENT     */ { 100,  20,   5,  20 },
    /* IMPACT      */ {  10,   2,   1,   2 },
};

static volatile fsw_state_t s_state = ST_BOOT;
static uint32_t             s_enter_ms;
static volatile bool s_payload_locked   = true;    /* cabinets are locked at power-on */
static volatile bool s_parachute_locked = true;
static QueueHandle_t        s_cmd_q;

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

const char *fsw_state_name(fsw_state_t s)
{
    return (s < ST_COUNT) ? STATE_NAME[s] : "?";
}

fsw_state_t fsw_get_state(void)  { return s_state; }
bool fsw_payload_locked(void)    { return s_payload_locked; }
bool fsw_parachute_locked(void)  { return s_parachute_locked; }
bool fsw_post_command(cmd_opcode_t op)
{
    uint8_t b = (uint8_t)op;
    return xQueueSend(s_cmd_q, &b, 0) == pdTRUE;
}

/* Send the new period to every rate-controlled task. */
static void apply_rates(fsw_state_t s)
{
    for (int t = 0; t < RT_COUNT; t++) {
        rate_set((rate_task_t)t, 1000u / RATE_HZ[s][t]);
    }
    ESP_LOGI(TAG, "rates: IMU=%u BARO=%u GNSS=%u DATA=%u Hz",
             RATE_HZ[s][RT_IMU], RATE_HZ[s][RT_BARO], RATE_HZ[s][RT_GNSS], RATE_HZ[s][RT_DATA]);
}

static void enter_state(fsw_state_t next)
{
    ESP_LOGI(TAG, "STATE %s -> %s", fsw_state_name(s_state), fsw_state_name(next));
    s_state    = next;
    s_enter_ms = now_ms();
    apply_rates(next);
    /* Stage 6: save `next` to the SD card here */

    switch (next) {
    case ST_ASCENT:
        if (!s_payload_locked || !s_parachute_locked) {
            ESP_LOGW(TAG, "launch with a cabinet UNLOCKED (payload=%d parachute=%d)",
                     s_payload_locked, s_parachute_locked);
        }
        break;
    case ST_PAYLOAD_SEP:                         /* release both, in the same tick as the transition */
        act_payload_unlock();
        act_parachute_unlock();
        s_payload_locked = false;
        s_parachute_locked = false;
        break;
    default:
        break;
    }
}

static void handle_command(uint8_t op)
{
    switch (op) {
    case CMD_ENTER_TEST:
        if (s_state == ST_BOOT) enter_state(ST_TEST_MODE);
        else ESP_LOGW(TAG, "ENTER_TEST ignored in %s", fsw_state_name(s_state));
        break;
    case CMD_ENTER_LAUNCHPAD:
        if (s_state == ST_TEST_MODE) enter_state(ST_LAUNCH_PAD);
        else ESP_LOGW(TAG, "ENTER_LAUNCHPAD ignored in %s", fsw_state_name(s_state));
        break;
    case CMD_TOGGLE_PAYLOAD:                     /* ground only */
        if (s_state <= ST_LAUNCH_PAD) {
            s_payload_locked = !s_payload_locked;
            if (s_payload_locked) act_payload_lock();
            else                  act_payload_unlock();
        } else {
            ESP_LOGW(TAG, "payload command REJECTED in %s", fsw_state_name(s_state));
        }
        break;
    case CMD_TOGGLE_PARACHUTE:                   /* ground only */
        if (s_state <= ST_LAUNCH_PAD) {
            s_parachute_locked = !s_parachute_locked;
            if (s_parachute_locked) act_parachute_lock();
            else                    act_parachute_unlock();
        } else {
            ESP_LOGW(TAG, "parachute command REJECTED in %s", fsw_state_name(s_state));
        }
        break;
    case CMD_TRIGGER_SEP:                        /* manual override, ASCENT only */
        if (s_state == ST_ASCENT) {
            ESP_LOGW(TAG, "manual separation command");
            enter_state(ST_PAYLOAD_SEP);
        } else {
            ESP_LOGW(TAG, "TRIGGER_SEP ignored in %s", fsw_state_name(s_state));
        }
        break;
    default:
        ESP_LOGW(TAG, "command 0x%02X not handled yet", op);
        break;
    }
}

static void evaluate(const abstract_state_t *a)
{
    uint32_t in_state = now_ms() - s_enter_ms;

    switch (s_state) {
    case ST_BOOT:
        if (a->valid) enter_state(ST_TEST_MODE);
        break;
    case ST_TEST_MODE:
        if (a->valid && in_state >= TEST_MODE_MIN_MS) enter_state(ST_LAUNCH_PAD);
        break;
    case ST_LAUNCH_PAD:
        if (a->launch_detected) enter_state(ST_ASCENT);
        break;
    case ST_ASCENT:
        if (a->apogee_detected) {
            enter_state(ST_PAYLOAD_SEP);
        } 
        break;
    case ST_PAYLOAD_SEP:
        if (in_state >= SEP_DWELL_MS) enter_state(ST_DESCENT);
        break;
    case ST_DESCENT:
        if (a->impact_detected) enter_state(ST_IMPACT);
        break;
    default:                                     /* ST_IMPACT: final state */
        break;
    }
}

static void fsm_task(void *arg)
{
    s_enter_ms = now_ms();
    ESP_LOGI(TAG, "starting in %s", fsw_state_name(s_state));
    apply_rates(s_state);
    act_payload_lock();                         
    act_parachute_lock();
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        uint8_t op;
        while (xQueueReceive(s_cmd_q, &op, 0) == pdTRUE) {
            handle_command(op);
        }

        abstract_state_t a;
        shared_read_abstract(&a);
        evaluate(&a);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

void fsw_core_start(void)
{
    act_init();
    s_cmd_q = xQueueCreate(8, sizeof(uint8_t));
    xTaskCreate(fsm_task, "fsm", 4096, NULL, 7, NULL);
}