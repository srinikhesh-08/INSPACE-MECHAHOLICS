#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Continuous-rotation servos have no position feedback, so the FSM tracks locked/unlocked.
   Unlocked = 0 deg, locked = 180 deg. Lock = forward timed move, unlock = reverse timed move.
   All calls return immediately; the move runs in the servo's own task. */
void act_init(void);                 /* sets up PWM and tasks. Moves nothing. */
void act_payload_lock(void);
void act_payload_unlock(void);
void act_parachute_lock(void);
void act_parachute_unlock(void);

#ifdef __cplusplus
}
#endif