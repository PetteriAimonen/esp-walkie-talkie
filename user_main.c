#include "espressif/esp_common.h"
#include "espressif/phy_info.h"
#include "esp/uart.h"
#include <esp/gpio.h>
#include "FreeRTOS.h"
#include <task.h>
#include <dhcpserver.h>
#include "audio_task.h"
#include "ssid_config.h"

#define GPIO_POWERON  16
#define GPIO_BUTTON1  14
#define GPIO_BUTTON2  12

static volatile int g_volume = 4;

int get_volume() { return g_volume; }

void pwr_task(void *pvParameters)
{
  int heldcount = 0;
  int prev_btns = 0;
  
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
    
    int btns = 0;
    if (gpio_read(GPIO_BUTTON1)) btns |= 1;
    if (gpio_read(GPIO_BUTTON2)) btns |= 2;
    if (prev_btns == 0 && btns != 0)
    {
      if (btns & 1)
      {
        if (g_volume > 1) g_volume--;
      }
      else if (btns & 2)
      {
        if (g_volume < 16) g_volume++;
      }
      
      vTaskDelay(10);
    }
    
    prev_btns = btns;
  }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    
    {
      sdk_phy_info_t phy_info;
      get_sdk_default_phy_info(&phy_info);
      
//       dump_phy_info(&phy_info, false);
/*      
      phy_info.pa_vdd = 37;
      
      phy_info.target_power_index_mcs[0] = 4;
      phy_info.target_power_index_mcs[1] = 4;
      phy_info.target_power_index_mcs[2] = 4;
      phy_info.target_power_index_mcs[3] = 4;
      phy_info.target_power_index_mcs[4] = 4;
      phy_info.target_power_index_mcs[5] = 4;
      phy_info.target_power_index_mcs[6] = 4;
      phy_info.target_power_index_mcs[7] = 4;
      */
      write_saved_phy_info(&phy_info);
    }

    // Enable poweron pin
    gpio_enable(GPIO_POWERON, GPIO_OUTPUT);
    gpio_enable(GPIO_BUTTON1, GPIO_INPUT);
    gpio_enable(GPIO_BUTTON2, GPIO_INPUT);
    gpio_write(GPIO_POWERON, 0);
    
    // Button must be held for 1 second to power on
    vTaskDelay(1 * configTICK_RATE_HZ);
    gpio_write(GPIO_POWERON, 1);
    
    while (gpio_read(GPIO_BUTTON1));
    
    xTaskCreate(pwr_task, "pwr", 256, NULL, 2, NULL);
    
    printf("Waiting to connect..\n");
    
        
    struct sdk_station_config config = {
        .ssid     = "ESP_FC7BAB",
        .password = "",
    };
    
    struct sdk_softap_config ap_config = {
        .ssid = "ESP_FC7BAB",
        .password = "",
        .ssid_len = 10,
        .channel = 6,
        .authmode = AUTH_OPEN,
        .ssid_hidden = 0,
        .max_connection = 3,
        .beacon_interval = 100
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    vTaskDelay(10);
    sdk_wifi_station_disconnect();
    vTaskDelay(10);
    sdk_wifi_station_connect();
    
    
//     while (maxwait-- > 0)
//     while(1)
//     {
//       vTaskDelay(configTICK_RATE_HZ);
//       
//       int status = sdk_wifi_station_get_connect_status();
//       printf("Status: %d\n", status);
//     }
    
//     if (sdk_wifi_station_get_connect_status() != STATION_GOT_IP)
//     {
//       sdk_wifi_softap_set_config(&ap_config);
//       sdk_wifi_set_opmode(SOFTAP_MODE);
//       vTaskDelay(configTICK_RATE_HZ);
//       
//       struct ip_info ap_ip;
//       IP4_ADDR(&ap_ip.ip, 172, 16, 0, 1);
//       IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
//       IP4_ADDR(&ap_ip.netmask, 255, 255, 0, 0);
//       sdk_wifi_set_ip_info(1, &ap_ip);
//       sdk_wifi_softap_set_config(&ap_config);
//       
//       vTaskDelay(configTICK_RATE_HZ);
//       ip_addr_t first_client_ip;
//       IP4_ADDR(&first_client_ip, 172, 16, 0, 2);
//       dhcpserver_start(&first_client_ip, 4);
//     }
    
    
    audio_task_start();
}
