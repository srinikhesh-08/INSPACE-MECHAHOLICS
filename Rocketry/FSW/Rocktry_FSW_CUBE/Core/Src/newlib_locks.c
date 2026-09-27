/*
 * newlib_locks.c
 *
 *  Created on: Sep 23, 2026
 *      Author: Don Chacko
 */

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t malloc_mutex = NULL;

void __malloc_lock(struct _reent *r) {
    if (malloc_mutex == NULL) {
        malloc_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(malloc_mutex, portMAX_DELAY);
}

void __malloc_unlock(struct _reent *r) {
    xSemaphoreGive(malloc_mutex);
}

