#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa UART y lanza la tarea (core 0)
void start_uart_task(int16_t *inputs, int16_t *outputs);

#ifdef __cplusplus
}
#endif
