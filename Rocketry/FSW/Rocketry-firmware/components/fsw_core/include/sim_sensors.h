#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the task that writes fake baro, IMU and GNSS data into the shared buffers. */
void sim_sensors_start(void);
void sim_request_launch(void);   /* console/test hook: start boost when the sim is sitting on the pad */
#ifdef __cplusplus
}
#endif