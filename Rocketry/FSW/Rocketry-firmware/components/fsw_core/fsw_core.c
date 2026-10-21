#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "actuators.h"
#include "fsw_core.h"
#include "persist.h"

static const char *TAG = "fsm";

static const char *const STATE_NAME[ST_COUNT] = {
    "BOOT", "TEST_MODE", "LAUNCH_PAD", "ASCENT", "PAYLOAD_SEP", "DESCENT", "IMPACT"
};

/* Task rates in Hz per state (spec table). Column order must match rate_task_t. */
static const uint16_t RATE_HZ[ST_COUNT][RT_COUNT] = {
    /*                 IMU  BARO GNSS DATA  LOG */
    /* BOOT        */ {  10,   5,   1,   5,   1 },
    /* TEST_MODE   */ {  10,   5,   1,   5,   1 },
    /* LAUNCH_PAD  */ {  10,   5,   5,   5,  1 },
    /* ASCENT      */ { 100,  20,   5,  20,  1 },
    /* PAYLOAD_SEP */ {  50,  10,   5,  10,  1 },
    /* DESCENT     */ { 100,  20,   5,  20,  1 },
    /* IMPACT      */ {  10,   2,   1,   2,   1 },
};

static volatile fsw_state_t s_state = ST_BOOT;
static uint32_t             s_enter_ms;
static volatile bool s_payload_locked   = true;    /* cabinets are locked at power-on */
static volatile bool s_parachute_locked = true;
static QueueHandle_t        s_cmd_q;
static bool  s_restored = false;
static float s_rest_p_ref = 0.0f;
static float s_rest_max_alt = 0.0f;

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
    ESP_LOGI(TAG, "rates: IMU=%u BARO=%u GNSS=%u DATA=%u LOG=%u Hz",
             RATE_HZ[s][RT_IMU], RATE_HZ[s][RT_BARO], RATE_HZ[s][RT_GNSS],
             RATE_HZ[s][RT_DATA], RATE_HZ[s][RT_LOG]);
}

static void save_record(void)
{
    abstract_state_t a;
    shared_read_abstract(&a);
    persist_record_t r = {};
    r.state            = (uint8_t)s_state;
    r.payload_locked   = s_payload_locked;
    r.parachute_locked = s_parachute_locked;
    r.p_ref_pa         = a.p_ref_pa;
    r.max_alt_m        = a.max_alt_m;
    persist_save(&r);
}

static void enter_state(fsw_state_t next)
{
    ESP_LOGI(TAG, "STATE %s -> %s", fsw_state_name(s_state), fsw_state_name(next));
    s_state    = next;
    s_enter_ms = now_ms();
    apply_rates(next);

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
    case ST_LAUNCH_PAD:
        if (!persist_ready()) persist_repair();   /* safe: we reached the pad by a normal boot */
        break;
    default:
        break;
    }
    save_record();
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
    case CMD_START_TELEMETRY:
        telemetry_set_tx(true);
        break;
    case CMD_STOP_TELEMETRY:
        telemetry_set_tx(false);
        break;
    case CMD_SWITCH_CHANNEL:
        telemetry_toggle_channel();
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
    persist_init();

    persist_record_t r;
    if (persist_load(&r) &&
        (r.state == ST_ASCENT || r.state == ST_PAYLOAD_SEP || r.state == ST_DESCENT)) {
        s_state             = (fsw_state_t)r.state;
        s_payload_locked    = r.payload_locked;
        s_parachute_locked  = r.parachute_locked;
        s_rest_p_ref        = r.p_ref_pa;
        s_rest_max_alt      = r.max_alt_m;
        s_restored          = true;
        ESP_LOGW(TAG, "RESET RECOVERY: resuming in %s", fsw_state_name(s_state));
    }

    s_cmd_q = xQueueCreate(8, sizeof(uint8_t));
    xTaskCreate(fsm_task, "fsm", 4096, NULL, 7, NULL);
    logger_start();
    telemetry_start();
}

bool fsw_restored_info(float *p_ref, float *max_alt)
{
    if (p_ref)   *p_ref   = s_rest_p_ref;
    if (max_alt) *max_alt = s_rest_max_alt;
    return s_restored;
}