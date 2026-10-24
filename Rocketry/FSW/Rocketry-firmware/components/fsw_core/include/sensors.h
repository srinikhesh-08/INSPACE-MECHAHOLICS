#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void baro_start(void);
void gnss_start(void);
void imu_start(void);
void imu_calibrate_gyro(void);
void imu_calibrate_accel(void);
void sensors_start(void);     /* creates the I2C bus, then starts baro, imu and gnss */
void  power_start(void);
float power_battery_v(void);       /* volts, 0.0 if the INA219 is missing */
bool  power_battery_ok(void);
bool  power_battery_low(void);     /* true after 5 s below VBAT_LOW_V */
#ifdef __cplusplus
}
#endif