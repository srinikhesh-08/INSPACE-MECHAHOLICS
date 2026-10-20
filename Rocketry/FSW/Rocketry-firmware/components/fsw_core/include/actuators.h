#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Servo positions: 180 deg = locked, 0 deg = unlocked/open.
   These must return immediately (set the target and go): the real PWM comes in a later stage. */
void act_payload_lock(void);
void act_payload_unlock(void);
void act_parachute_lock(void);
void act_parachute_unlock(void);
void act_init(void);    /* call once at startup: sets up PWM and drives both servos to LOCKED */
#ifdef __cplusplus
}
#endif