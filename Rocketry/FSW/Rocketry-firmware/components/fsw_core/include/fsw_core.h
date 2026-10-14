#pragma once
#include <stdbool.h>
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Creates the command queue and the FSM task (10 Hz). */
void fsw_core_start(void);

/* Telecommand entry point: the comm layer (and the debug console) call this from any task. */
bool fsw_post_command(cmd_opcode_t op);

fsw_state_t fsw_get_state(void);
const char *fsw_state_name(fsw_state_t s);
bool        fsw_payload_locked(void);
bool        fsw_parachute_locked(void);

#ifdef __cplusplus
}
#endif