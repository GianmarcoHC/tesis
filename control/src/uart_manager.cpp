#include "uart_manager.h"
#include <string.h>
#include <stdio.h>

#define UART_PORT UART_NUM_0
#define BUF_SIZE  1024

// punteros a las variables del main
static int16_t *rx_data_ptr = nullptr; // MATLAB → ESP32
static int16_t *tx_data_ptr = nullptr; // ESP32 → MATLAB

static void uart_task(void *pvParameters)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t rx_buffer[BUF_SIZE];
    uint8_t tx_buffer[11];

    while (true)
    {
        // === 1. Enviar 4 int16_t (ESP32 → PC) ===
        tx_buffer[0] = 'R';
        memcpy(&tx_buffer[1], tx_data_ptr, 8);
        tx_buffer[9] = '\n';
        uart_write_bytes(UART_PORT, (const char *)tx_buffer, 10);

        // === 2. Leer 4 int16_t (PC → ESP32) ===
        int len = uart_read_bytes(UART_PORT, rx_buffer, 10, pdMS_TO_TICKS(10));
        if (len >= 10 && rx_buffer[0] == 'H' && rx_buffer[9] == '\n')
        {
            memcpy(rx_data_ptr, &rx_buffer[1], 8);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // ~100 Hz
    }
}

void start_uart_task(int16_t *inputs, int16_t *outputs)
{
    rx_data_ptr = inputs;
    tx_data_ptr = outputs;
    xTaskCreatePinnedToCore(uart_task, "UART_Task", 4096, NULL, 5, NULL, 0);
}
