/*
 * blinker.c
 *
 *  Created on: Sep 25, 2026
 *      Author: Don Chacko
 */

#include "blinker.h"
#include "main.h"

static GPIO_PinState ledState = GPIO_PIN_RESET;  // RESET = LED on (sink mode)

void heartbeat_Init(void)
{
    ledState = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ledState);
}

void heartbeat_Toggle(void)
{
    ledState = (ledState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ledState);
}
