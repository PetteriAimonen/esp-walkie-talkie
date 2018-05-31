/* Very basic example that just demonstrates we can run at all!
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include <esp/gpio.h>
#include "FreeRTOS.h"
#include <task.h>
#include "audio_task.h"
#include "ssid_config.h"

#define GPIO_POWERON  16
#define GPIO_BUTTON1  14

void pwr_task(void *pvParameters)
{
  int heldcount = 0;
  
  while(1)
  {
    vTaskDelay(1);
    
    if (gpio_read(GPIO_BUTTON1))
    {
      heldcount++;
    }
    else
    {
      heldcount = 0;
    }
    
    if (heldcount > configTICK_RATE_HZ)
    {
      // Power off
      printf("Powering off..\n");
      vTaskDelay(1);
      gpio_write(GPIO_POWERON, 0);
    }
  }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    
    struct sdk_station_config config = {
        .ssid     = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    
    // Enable poweron pin
    gpio_enable(GPIO_POWERON, GPIO_OUTPUT);
    gpio_enable(GPIO_BUTTON1, GPIO_INPUT);
    gpio_write(GPIO_POWERON, 0);
    
    // Button must be held for 1 second to power on
    vTaskDelay(1 * configTICK_RATE_HZ);
    gpio_write(GPIO_POWERON, 1);
    
    while (gpio_read(GPIO_BUTTON1));
    
    xTaskCreate(pwr_task, "pwr", 256, NULL, 2, NULL);
    
    audio_task_start();
}
