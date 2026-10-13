#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the task that writes fake baro, IMU and GNSS data into the shared buffers. */
void sim_sensors_start(void);

#ifdef __cplusplus
}
#endif