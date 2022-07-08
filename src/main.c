#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#define BLINK_GPIO 2
#define BUF_SIZE (1024)
#define ECHO_UART_PORT_NUM 0
#define ECHO_TEST_RXD 3
#define ECHO_TEST_TXD 1
#define ECHO_TEST_RTS 22
#define ECHO_TEST_CTS 19
#define ReverseUInt(value) \
((value >> 24) | ((value >> 16) << 8) | ((value & 0xff00) << 8) | ((value& 0xff) << 24))

#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_6
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11
static esp_adc_cal_characteristics_t adc1_chars;
uint32_t  voltage=0;
static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;
 printf("adc Init...\n");
    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        printf( "Calibration scheme not supported, skip software calibration\n");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        printf("eFuse not burnt, skip software calibration\n");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
 printf("adc Init ok\n");
    } else {
        printf("Invalid arg\n");
    }

    return cali_enable;
}

//====================================================================================================================
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

uint32_t crc32(unsigned char *message, int len)
{
    if ((message == 0) || (len < 1)) {
        return 0;
    }
    unsigned int byte, crc;
    crc = 0xFFFFFFFF;
    for (int i=0; i<len; i++) {
        byte = message[i];
        byte = ReverseUInt(byte);
        for (int j=0; j<8; j++) {
            if (((int)(crc ^ byte)) < 0) {
                crc = (crc << 1) ^ 0x04C11DB7;
            } else {
                crc = crc << 1;
            }
            byte = byte << 1;
        }
    }
    return ReverseUInt(~crc);
}
//====================================================================================================================
void SendTemp(uint32_t temp)
{uint8_t packet[50]={0x0a,0x0d,0x10,0x05,0x00,0x28,
                    0x00,0x0a,0x00,0x01,0x00,0x01,0x00,0x1e,0x00,0x00,
                    0x00,0x0a,0x00,0x02,0x00,0x01,0x00,0x00,0x00,0x00,
                    0x00,0x0a,0x00,0x03,0x00,0x01,0x00,0x28,0x00,0x00,
                    0x00,0x0a,0x00,0x05,0x00,0x01,0x00,0x3c,0x00,0x00,
                    0xb6,0xd6,0x9a,0x42};
    uint32_t crc=0;
    packet[22]=(temp>>8)&0xff;
    packet[23]=(temp>>0)&0xff;
    crc= crc32(packet,46);
    packet[46]=(crc>>24)&0xff;
    packet[47]=(crc>>16)&0xff;
    packet[48]=(crc>>8)&0xff;
    packet[49]=(crc>>0)&0xff;
      uart_write_bytes(ECHO_UART_PORT_NUM, packet, sizeof(packet));
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
    adc_calibration_init();
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
    while(1) {
        /* Blink off (output low) */
	// printf("Turning off the LED\n");
    voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_EXAMPLE_CHAN0), &adc1_chars);
    printf("v= %d mv\n",voltage);


         gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        // SendTemp(22);
	// printf("Turning on the LED\n");
        gpio_set_level(BLINK_GPIO, 1);
     
      

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
