#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define BLINK_GPIO 2
#define BUF_SIZE (1024)
#define ECHO_UART_PORT_NUM 0
#define ECHO_TEST_RXD 3
#define ECHO_TEST_TXD 1
#define ECHO_TEST_RTS 22
#define ECHO_TEST_CTS 19
void UARTInit()
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
     ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
     ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

}


void app_main()
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    UARTInit();
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while(1) {
        /* Blink off (output low) */
	// printf("Turning off the LED\n");
     int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
	// printf("Turning on the LED\n");
        gpio_set_level(BLINK_GPIO, 1);
        if(len>0)
        uart_write_bytes(ECHO_UART_PORT_NUM, data, len);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
