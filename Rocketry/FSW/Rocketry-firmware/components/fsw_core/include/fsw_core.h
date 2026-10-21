#pragma once
#include <stdbool.h>
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Creates the command queue and the FSM task (10 Hz). */
void fsw_core_start(void);
/* Starts the SD logger task (call after shared_init). */
void logger_start(void);
void telemetry_start(void);
void telemetry_set_tx(bool on);
void telemetry_toggle_channel(void);
bool telemetry_tx_enabled(void);
bool telemetry_on_backup(void);
bool logger_post_line(const char *line);
/* Telecommand entry point: the comm layer (and the debug console) call this from any task. */
bool fsw_post_command(cmd_opcode_t op);

fsw_state_t fsw_get_state(void);
const char *fsw_state_name(fsw_state_t s);
bool        fsw_payload_locked(void);
bool        fsw_parachute_locked(void);
/* true if the FSM resumed from a saved state after a reset */
bool fsw_restored_info(float *p_ref, float *max_alt);   /* pointers may be NULL */
#ifdef __cplusplus
}
#endif