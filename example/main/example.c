#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dalybms.h"
#include "sdkconfig.h"

static const char *TAG = "DALYBMS TEST";

void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(
        uart_set_pin(
            uart_num,
            CONFIG_UART_TX,
            CONFIG_UART_RX,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    ESP_ERROR_CHECK(
        uart_driver_install(
            uart_num,
            CONFIG_UART1_RX_BUFSIZE,
            CONFIG_UART1_TX_BUFSIZE,
            0,
            NULL,
            0
        )
    );
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    dalybms_test(uart_num);
}

