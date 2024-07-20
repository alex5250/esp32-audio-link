
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

void start_audio_link(void *pvParameters);
void init_sd_card(void);
void init_adc_buttons();

