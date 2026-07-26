#pragma once
#include "core_uart_receiver.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void job_dispatcher_init(void);
void job_dispatcher_on_command(core_command_t cmd);
void job_dispatcher_on_event(core_event_t evt);

// Already implemented in job_dispatcher.c — was just not declared here.
// Used by button_handler.c to avoid showing a confirm prompt (or
// dispatching) while a command is already running.
bool job_dispatcher_is_busy(void);

#ifdef __cplusplus
}
#endif