

#include "esp32-wifi-raw.h"
#include "fec.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "string.h"
#include "microphone.h"
#include "g72x.h"


void vSendDataFromMic( void * pvParameters );



char msg[10] = "any packet!!!!";  //message to be transmitted
char message[60];
 //message length
#define packet_size 64
int j = 0;

void app_main() {

    struct g72x_state state = {
        .yl = 0,
        .yu = 0,
        .dms = 0,
        .dml = 0,
        .ap = 0,
        .a = {0, 0},
        .b = {0, 0, 0, 0, 0, 0},
        .pk = {0, 0},
        .dq = {0, 0, 0, 0, 0, 0},
        .sr = {0, 0},
        .td = 0
    };

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  init_esp32_wifi_raw();
  init_microphone();
  g72x_init_state(&state);
  
  
  xTaskCreatePinnedToCore(vSendDataFromMic, "Task", 4096, NULL, 1, NULL, 0);


  
}

void vSendDataFromMic( void * pvParameters )
{
for( ;; )
  {
    uint16_t data_from_mic[packet_size];
    getDataFromMicrophone(data_from_mic,packet_size);
    g721_encoder(data_from_mic,2,&state)
    encode_rs_8((uint8_t *) &data_from_mic[0], (uint8_t *) &data_from_mic[packet_size], (223 - packet_size));
  }
}



 



   
  



