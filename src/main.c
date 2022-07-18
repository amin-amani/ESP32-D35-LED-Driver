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
#include "nvs_flash.h"
#include "nvs.h"
#include "Buttons.h"
#define BLINK_GPIO 2
#define GPIO_INPUT_IO_0     12
#define GPIO_INPUT_IO_1     14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))


#define BUF_SIZE (1024)
#define ECHO_UART_PORT_NUM 0
#define ECHO_TEST_RXD 3
#define ECHO_TEST_TXD 1
#define ECHO_TEST_RTS 22
#define ECHO_TEST_CTS 19
#define ADC1_EXAMPLE_CHAN0          ADC2_CHANNEL_3 
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

#define ReverseUInt(value) \
((value >> 24) | ((value >> 16) << 8) | ((value & 0xff00) << 8) | ((value& 0xff) << 24))
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static esp_adc_cal_characteristics_t adc1_chars;
uint32_t  voltage=0;
int v1;
double temp=0,AvrageVoltage=0;
int32_t ramp=100;
//====================================================================================================================

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
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
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
//====================================================================================================================

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

void NVSInit()
{
  esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    

}
//====================================================================================================================

void SetRampParameter(int32_t value)
{
         nvs_handle_t my_handle;

   esp_err_t  err = nvs_open("storage", NVS_READWRITE, &my_handle);
       if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return ;
    }
            err = nvs_set_i32(my_handle, "ramp", value);
        if(err != ESP_OK)
        {
            printf("save param Failed!\n");
           
           
        }  
            nvs_close(my_handle);
}
//====================================================================================================================

uint32_t GetRampParameter()
{
  int32_t result = 100; 
     nvs_handle_t my_handle;
   esp_err_t  err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return result;
    }
     

      // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "ramp", &result);
        switch (err) {
            case ESP_OK:

                printf("Restart ramp = %d\n", result);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        nvs_close(my_handle);
        return result;
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
//====================================================================================================================
 bool Key1()
{
    return gpio_get_level(GPIO_INPUT_IO_0);
    
}
//====================================================================================================================
 bool Key2()
{
return gpio_get_level(GPIO_INPUT_IO_1);
//	return ((GPIOB->IDR&(1<<4))>>4);
}
//====================================================================================================================
void OnUpKeyPressed()
{
printf("Up\n");
if(ramp<120)
ramp++;
SetRampParameter(ramp);

}
//====================================================================================================================

void OnDownKeyPressed()
{
printf("down\n");

if(ramp>80)
ramp--;
SetRampParameter(ramp);
}

//====================================================================================================================
void GPIOInit()
{
        gpio_config_t io_conf = {};
   //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
      
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

}
void app_main()
{
    GPIOInit();
    UARTInit();
    NVSInit();
    ramp=GetRampParameter();
    adc_calibration_init();
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    RegisterButton(Key1,OnUpKeyPressed,2);
    RegisterButton(Key2,OnDownKeyPressed,2);
    while(1) 
    {
  
    adc2_get_raw(ADC1_EXAMPLE_CHAN0,ADC_WIDTH_12Bit,&v1);
    voltage = esp_adc_cal_raw_to_voltage(v1, &adc1_chars);

    AvrageVoltage=(AvrageVoltage*9 +voltage)/10;
    temp=AvrageVoltage;
    temp=(145*temp*ramp)/100000; //60 to 140
    temp-=87;
    SendTemp(temp);
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    CheckButtons();
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
