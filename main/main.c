#include "audio-link.h"



void app_main(void)
{

    init_adc_buttons();
    init_sd_card();

   xTaskCreate(start_audio_link, "audio-link", 32768, NULL, 1, NULL);
}
