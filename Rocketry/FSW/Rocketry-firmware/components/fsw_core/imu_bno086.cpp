/* BNO086 IMU driver for the FSW.
   The HAL, the soft reset and the bus settings are the ones that worked on the real sensor.
   Around them: shared I2C bus, shared-state publishing, gyro/accelerometer calibration. */
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c_master.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "i2c_bus.h"
#include "sensors.h"

extern "C" {                      /* the SH2 headers have no C++ guards: this wrapper is required */
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
}
#ifndef SH2_OK
#define SH2_OK 0          /* the SH2 library documents "SH2_OK (0)" but never defines it */
#endif
static const char* tag = "BNO086";

/* ---------------------------------------------------------------- */
/* Settings                                                         */
/* ---------------------------------------------------------------- */
#define IMU_I2C_ADDR       0x4B          /* your sensor answered here */
#define IMU_STALE_MS       500           /* no data for this long => publish invalid */
#define IMU_RESTART_MS     2000          /* silent this long => ask for the reports again */
#define IMU_MIN_PERIOD_MS  10            /* fastest report rate: 10 ms = 100 Hz; raise to 20 if the CPU load is too high */
#define RAD_TO_DEG         57.2957795f

static i2c_master_dev_handle_t dev_handle = nullptr;
static bool s_hal_got_packet = false;    /* set by hal_read when it delivered a packet */

static imu_data_t    s_imu = {};         /* what the rest of the FSW reads (through shared state) */
static bool          s_have_accel = false;
static bool          s_have_gyro  = false;
static volatile bool s_need_config = true;
static uint32_t      s_last_event_ms = 0;

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ---------------------------------------------------------------- */
/* SH2 HAL (your working code)                                      */
/* ---------------------------------------------------------------- */
static int hal_open(sh2_Hal_t *self)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Send SHTP Executable Channel soft-reset packet since there is no RST pin
    uint8_t soft_reset_pkt[5] = { 5, 0, 1, 0, 1 };
    i2c_master_transmit(dev_handle, soft_reset_pkt, 5, 200);

    // Wait for BNO086 internal firmware to reboot
    vTaskDelay(pdMS_TO_TICKS(300));
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
}

static uint32_t hal_gettime(sh2_Hal_t *self)
{
    return (uint32_t)esp_timer_get_time();
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    // Give sensor brief breathing room before transmitting
    esp_rom_delay_us(1000);
    if (i2c_master_transmit(dev_handle, pBuffer, len, 200) != ESP_OK)
    {
        return 0;
    }
    return len;
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    uint8_t header[4] = {0};

    // Prevent sh2 internal blocking loops from slamming the I2C bus with 0 delay
    esp_rom_delay_us(2000);

    // 1. Read the 4-byte SHTP header (200ms timeout avoids 1-tick mid-byte aborts)
    if (i2c_master_receive(dev_handle, header, 4, 200) != ESP_OK)
    {
        return 0;
    }

    // 2. Extract total packet length (bytes 0 and 1, masking out continuation bit 15)
    uint16_t packet_len = ((uint16_t)(header[1] & 0x7F) << 8) | header[0];

    // If length is 0, <= 4 (header only), or 0x7FFF (0xFFFF idle bus), no payload is waiting
    if (packet_len <= 4 || packet_len == 0x7FFF)
    {
        return 0;
    }

    *t_us = (uint32_t)esp_timer_get_time();

    // 3. Cap read size to the buffer size provided by sh2
    uint16_t bytes_to_read = (packet_len > len) ? len : packet_len;

    // Give sensor 500us before starting the payload transaction
    esp_rom_delay_us(500);

    // 4. Every I2C read starts with a 4-byte SHTP header + (bytes_to_read - 4) payload bytes.
    // Read the full 'bytes_to_read' so no trailing bytes are left stuck in the sensor FIFO.
    if (i2c_master_receive(dev_handle, pBuffer, bytes_to_read, 200) != ESP_OK)
    {
        return 0;
    }

    // 5. Reconstruct the SHTP header in pBuffer[0..3] with the actual read length
    // while preserving the original continuation bit from the first header read.
    pBuffer[0] = bytes_to_read & 0xFF;
    pBuffer[1] = ((bytes_to_read >> 8) & 0x7F) | (header[1] & 0x80);
    pBuffer[2] = header[2];
    pBuffer[3] = header[3];

    s_hal_got_packet = true;      /* lets the task keep reading while data keeps arriving */
    return bytes_to_read;
}

static sh2_Hal_t imu_hal =
{
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_gettime
};

/* ---------------------------------------------------------------- */
/* Calibration: software offsets, applied to every published sample */
/* ---------------------------------------------------------------- */
#define CAL_DURATION_MS    3000      /* the rocket must stay still this long */
#define CAL_MIN_SAMPLES    15
#define GYRO_MAX_STD_DPS   1.0f      /* still enough: spread of each gyro axis */
#define GYRO_MAX_BIAS_DPS  15.0f     /* a bias larger than this means something is wrong */
#define ACC_MAX_STD        0.3f      /* m/s^2 */
#define ACC_MAX_TILT       1.5f      /* m/s^2 on x and y: about 9 degrees off vertical */
#define ACC_MAX_G_ERROR    1.5f      /* m/s^2: z must read about +-9.8 */
#define STD_GRAVITY        9.80665f

enum { CAL_NONE = 0, CAL_GYRO = 1, CAL_ACCEL = 2 };
static volatile int s_cal_request = CAL_NONE;     /* set by the FSM task */
static volatile int s_cal_mode    = CAL_NONE;     /* running now, set by the IMU task */
static uint32_t     s_cal_start_ms = 0;
static int          s_cal_n = 0;
static float        s_cal_sum[3]   = {0, 0, 0};
static float        s_cal_sumsq[3] = {0, 0, 0};

static float s_gyro_off[3] = {0, 0, 0};           /* deg/s, subtracted from every gyro sample */
static float s_acc_off[3]  = {0, 0, 0};           /* m/s^2, subtracted from every accelerometer sample */

static void cal_add(const float v[3])
{
    for (int k = 0; k < 3; k++) {
        s_cal_sum[k]   += v[k];
        s_cal_sumsq[k] += v[k] * v[k];
    }
    s_cal_n++;
}

static void cal_finish(int mode)
{
    if (s_cal_n < CAL_MIN_SAMPLES) {
        ESP_LOGW(tag, "calibration FAILED: only %d samples in %d ms", s_cal_n, CAL_DURATION_MS);
        return;
    }
    float mean[3], sd[3];
    for (int k = 0; k < 3; k++) {
        mean[k] = s_cal_sum[k] / s_cal_n;
        float var = s_cal_sumsq[k] / s_cal_n - mean[k] * mean[k];
        sd[k] = sqrtf(var > 0.0f ? var : 0.0f);
    }

    if (mode == CAL_GYRO) {
        if (sd[0] > GYRO_MAX_STD_DPS || sd[1] > GYRO_MAX_STD_DPS || sd[2] > GYRO_MAX_STD_DPS) {
            ESP_LOGW(tag, "gyro calibration FAILED: the rocket moved (spread %.2f %.2f %.2f deg/s)",
                     sd[0], sd[1], sd[2]);
            return;
        }
        if (fabsf(mean[0]) > GYRO_MAX_BIAS_DPS || fabsf(mean[1]) > GYRO_MAX_BIAS_DPS ||
            fabsf(mean[2]) > GYRO_MAX_BIAS_DPS) {
            ESP_LOGW(tag, "gyro calibration FAILED: bias too large (%.1f %.1f %.1f deg/s)",
                     mean[0], mean[1], mean[2]);
            return;
        }
        for (int k = 0; k < 3; k++) s_gyro_off[k] = mean[k];
        ESP_LOGI(tag, "gyro calibrated: offsets %.3f %.3f %.3f deg/s (%d samples)",
                 mean[0], mean[1], mean[2], s_cal_n);
    } else {
        if (sd[0] > ACC_MAX_STD || sd[1] > ACC_MAX_STD || sd[2] > ACC_MAX_STD) {
            ESP_LOGW(tag, "accelerometer calibration FAILED: the rocket moved (spread %.2f %.2f %.2f m/s2)",
                     sd[0], sd[1], sd[2]);
            return;
        }
        float g_expected = copysignf(STD_GRAVITY, mean[2]);   /* z carries gravity, either sign */
        if (fabsf(mean[0]) > ACC_MAX_TILT || fabsf(mean[1]) > ACC_MAX_TILT ||
            fabsf(mean[2] - g_expected) > ACC_MAX_G_ERROR) {
            ESP_LOGW(tag, "accelerometer calibration FAILED: keep the rocket vertical, nose up (mean %.2f %.2f %.2f m/s2)",
                     mean[0], mean[1], mean[2]);
            return;
        }
        s_acc_off[0] = mean[0];                  /* x and y should read 0 when vertical */
        s_acc_off[1] = mean[1];
        s_acc_off[2] = mean[2] - g_expected;     /* z should read exactly +-9.80665 */
        ESP_LOGI(tag, "accelerometer calibrated: offsets %.3f %.3f %.3f m/s2 (%d samples)",
                 s_acc_off[0], s_acc_off[1], s_acc_off[2], s_cal_n);
    }
}

/* ---------------------------------------------------------------- */
/* Sensor data and reset handlers                                   */
/* ---------------------------------------------------------------- */
static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;

    if (value.sensorId == SH2_ACCELEROMETER)                 /* m/s^2, gravity included */
    {
        float raw[3] = { value.un.accelerometer.x, value.un.accelerometer.y, value.un.accelerometer.z };
        if (s_cal_mode == CAL_ACCEL) cal_add(raw);
        s_imu.ax = raw[0] - s_acc_off[0];
        s_imu.ay = raw[1] - s_acc_off[1];
        s_imu.az = raw[2] - s_acc_off[2];
        s_have_accel = true;
        s_imu.t_ms = now_ms();
        s_last_event_ms = s_imu.t_ms;
        s_imu.valid = s_have_accel && s_have_gyro;
        shared_write_imu(&s_imu);                            /* publish on every accelerometer sample */
    }
    else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED)     /* rad/s -> deg/s */
    {
        float raw[3] = { value.un.gyroscope.x * RAD_TO_DEG,
                         value.un.gyroscope.y * RAD_TO_DEG,
                         value.un.gyroscope.z * RAD_TO_DEG };
        if (s_cal_mode == CAL_GYRO) cal_add(raw);
        s_imu.gx = raw[0] - s_gyro_off[0];
        s_imu.gy = raw[1] - s_gyro_off[1];
        s_imu.gz = raw[2] - s_gyro_off[2];
        s_have_gyro = true;
    }
}

static void reset_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    if (event->eventId == SH2_RESET) {
        ESP_LOGW(tag, "BNO086 reset, enabling reports again");
        s_need_config = true;                    /* the task re-applies the report settings */
    }
}

static void enable_reports(uint32_t period_ms)
{
    sh2_SensorConfig_t cfg = {};
    cfg.reportInterval_us = period_ms * 1000u;
    sh2_setSensorConfig(SH2_ACCELEROMETER, &cfg);
    sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
}

/* ---------------------------------------------------------------- */
/* The task                                                         */
/* ---------------------------------------------------------------- */
static void imu_task(void *arg)
{
    bool ok = false;
    i2c_master_bus_handle_t bus = i2c_bus_handle();
    if (bus != nullptr && i2c_master_probe(bus, IMU_I2C_ADDR, 100) == ESP_OK) {
        i2c_device_config_t dc = {};
        dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dc.device_address  = IMU_I2C_ADDR;
        dc.scl_speed_hz    = 100000;         /* the speed your working test used */
        dc.scl_wait_us     = 10000;          /* the BNO086 stretches the clock: allow up to 10 ms */
        ok = (i2c_master_bus_add_device(bus, &dc, &dev_handle) == ESP_OK) &&
             (sh2_open(&imu_hal, reset_handler, nullptr) == SH2_OK) &&
             (sh2_setSensorCallback(sensor_event_handler, nullptr) == SH2_OK);
    }
    if (!ok) {
        ESP_LOGE(tag, "BNO086 init failed (nothing answers at address 0x%02X)", IMU_I2C_ADDR);
    }

    rate_register(RT_IMU);
    uint32_t period_ms = 20;                 /* the FSM sends the real value right away */
    uint32_t last_restart_ms = 0;
    bool warned = false;

    while (true) {
        if (ok) {
            if (s_need_config) {
                s_need_config = false;
                enable_reports(period_ms);
            }

            int n = 0;                       /* no INT pin: keep reading while packets keep coming */
            do {
                s_hal_got_packet = false;
                sh2_service();
            } while (s_hal_got_packet && ++n < 8);

            uint32_t p = rate_poll(period_ms);   /* new rate from the FSM? */
            if (p < IMU_MIN_PERIOD_MS) p = IMU_MIN_PERIOD_MS;
            if (p != period_ms) {
                period_ms = p;
                s_need_config = true;        /* re-send the report interval next loop */
            }

            /* calibration: start when asked, finish after CAL_DURATION_MS of still data */
            if (s_cal_mode == CAL_NONE && s_cal_request != CAL_NONE) {
                s_cal_mode = s_cal_request;
                s_cal_request = CAL_NONE;
                s_cal_n = 0;
                for (int k = 0; k < 3; k++) { s_cal_sum[k] = 0; s_cal_sumsq[k] = 0; }
                s_cal_start_ms = now_ms();
                ESP_LOGI(tag, "%s calibration started: keep the rocket still for %d s",
                         s_cal_mode == CAL_GYRO ? "gyro" : "accelerometer", CAL_DURATION_MS / 1000);
            }
            if (s_cal_mode != CAL_NONE && (now_ms() - s_cal_start_ms) >= CAL_DURATION_MS) {
                cal_finish(s_cal_mode);
                s_cal_mode = CAL_NONE;
            }

            uint32_t now = now_ms();
            if (s_imu.valid && (now - s_last_event_ms) > IMU_STALE_MS) {
                s_imu.valid = false;         /* sensor went quiet */
                shared_write_imu(&s_imu);
            }

            /* silent for a while: the sensor may have restarted, so ask for the reports again */
            if ((now - s_last_event_ms) > IMU_RESTART_MS && (now - last_restart_ms) > IMU_RESTART_MS) {
                last_restart_ms = now;
                s_need_config = true;
            }
            if (!warned && s_last_event_ms == 0 && now > 8000) {
                warned = true;
                ESP_LOGE(tag, "BNO086 answers on I2C but sends no data");
            }
        }
        vTaskDelay(1);
    }
}

/* ---------------------------------------------------------------- */
/* Public functions (declared in sensors.h)                         */
/* ---------------------------------------------------------------- */
extern "C" void imu_start(void)
{
    xTaskCreate(imu_task, "imu", 6144, NULL, 6, NULL);
}

extern "C" void imu_calibrate_gyro(void)
{
    if (s_cal_mode != CAL_GYRO) s_cal_request = CAL_GYRO;    /* repeats of the same command are ignored */
}

extern "C" void imu_calibrate_accel(void)
{
    if (s_cal_mode != CAL_ACCEL) s_cal_request = CAL_ACCEL;
}