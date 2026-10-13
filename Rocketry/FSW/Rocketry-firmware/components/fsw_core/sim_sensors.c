#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared_state.h"
#include "sim_sensors.h"

#define G 9.81f

static const char *TAG = "sim";

typedef enum { PH_PAD, PH_BOOST, PH_COAST, PH_DESCENT, PH_LANDED } sim_phase_t;

/* Tiny deterministic noise generator (xorshift32): same noise on every run. */
static uint32_t s_rng = 12345u;
static float noise(float amp)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    float u = (float)(s_rng & 0xFFFFu) / 65535.0f;   /* 0..1 */
    return (2.0f * u - 1.0f) * amp;                  /* -amp..+amp */
}

static void sim_task(void *arg)
{
    const float dt = SIM_PERIOD_MS / 1000.0f;
    const uint32_t gnss_every = 1000u / SIM_PERIOD_MS;   /* GNSS at 1 Hz */

    sim_phase_t ph = PH_PAD;
    float alt = 0.0f, vel = 0.0f, t_phase = 0.0f;
    uint32_t tick = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        float a_net  = 0.0f;   /* true vertical acceleration */
        float a_spec = G;      /* what the accelerometer reads on the z axis */

        switch (ph) {
        case PH_PAD:
            break;
        case PH_BOOST:
            a_net  = SIM_BOOST_ACCEL_MPS2;
            a_spec = a_net + G;
            break;
        case PH_COAST:                       /* free fall: accelerometer reads about 0 */
            a_net  = -G;
            a_spec = 0.0f;
            break;
        case PH_DESCENT:                     /* ease toward steady parachute speed */
            a_net  = (-SIM_DESCENT_MPS - vel) / 1.0f;
            a_spec = a_net + G;
            break;
        case PH_LANDED:                      /* short impact spike, then at rest */
            a_spec = (t_phase < 0.1f) ? 45.0f : G;
            break;
        }

        /* integrate motion */
        if (ph != PH_PAD && ph != PH_LANDED) {
            vel += a_net * dt;
            alt += vel * dt;
        }
        t_phase += dt;

        /* phase transitions */
        sim_phase_t next = ph;
        if (ph == PH_PAD && t_phase >= SIM_PAD_S)        next = PH_BOOST;
        if (ph == PH_BOOST && t_phase >= SIM_BOOST_S)    next = PH_COAST;
        if (ph == PH_COAST && vel <= 0.0f)               next = PH_DESCENT;
        if (ph == PH_DESCENT && alt <= 0.0f) {
            alt = 0.0f;
            vel = 0.0f;
            next = PH_LANDED;
        }
        if (next != ph) {
            ESP_LOGI(TAG, "phase %d -> %d  alt=%.1f m  vel=%.1f m/s",
                     (int)ph, (int)next, alt, vel);
            ph = next;
            t_phase = 0.0f;
        }

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* ---- fake IMU ---- */
        imu_data_t imu = {};
        imu.t_ms  = now_ms;
        imu.ax    = noise(0.05f);
        imu.ay    = noise(0.05f);
        imu.az    = a_spec + noise(0.10f);
        imu.gx    = noise(0.5f);
        imu.gy    = noise(0.5f);
        imu.gz    = SIM_SPIN_DPS + noise(0.5f);
        imu.valid = true;
        shared_write_imu(&imu);

        /* ---- fake barometer (standard atmosphere) ---- */
        baro_data_t baro = {};
        baro.t_ms        = now_ms;
        baro.pressure_pa = SIM_P0_PA * powf(1.0f - alt / 44330.0f, 5.255f) + noise(3.0f);
        baro.temp_c      = 25.0f - 0.0065f * alt;
        baro.valid       = true;
        shared_write_baro(&baro);

        /* ---- fake GNSS, 1 Hz ---- */
        if (tick % gnss_every == 0) {
            gnss_data_t gnss = {};
            gnss.t_ms       = now_ms;
            gnss.utc_hhmmss = 120000u + (tick / gnss_every);   /* crude clock, fine for a sim */
            gnss.lat        = 13.0000f;                         /* placeholder coordinates */
            gnss.lon        = 77.0000f;
            gnss.alt_msl    = SIM_PAD_ELEV_M + alt;
            gnss.sats       = 3;
            gnss.fix        = true;
            gnss.valid      = true;
            shared_write_gnss(&gnss);
        }

        tick++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SIM_PERIOD_MS));
    }
}

void sim_sensors_start(void)
{
    xTaskCreate(sim_task, "sim", 4096, NULL, 6, NULL);
}