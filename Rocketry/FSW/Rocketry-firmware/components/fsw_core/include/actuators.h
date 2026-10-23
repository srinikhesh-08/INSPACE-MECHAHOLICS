#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Servo positions: 180 deg = locked, 0 deg = unlocked/open.
   These must return immediately (set the target and go): the real PWM comes in a later stage. */
void act_payload_lock(void);
void act_payload_unlock(void);
void act_parachute_lock(void);
void act_parachute_unlock(void);
void act_init(bool payload_locked, bool parachute_locked);   /* call once at startup, after the saved state is loaded: sets up PWM with each servo at its starting position */
#ifdef __cplusplus
}
#endif