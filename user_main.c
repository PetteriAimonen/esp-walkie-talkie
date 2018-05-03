/* Very basic example that just demonstrates we can run at all!
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "audio_task.h"

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_set_phy_mode(PHY_MODE_11B);
    sdk_wifi_set_channel(6);
    
    audio_task_start();
}
