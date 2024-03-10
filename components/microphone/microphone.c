#include <stdio.h>
#include "microphone.h"


#include "driver/i2s.h"
#include "esp_log.h"


#define I2S_SAMPLE_RATE 8000
#define DMA_BUFFER_LEN 1024
#define TICKS_TO_WAIT 1000
#define NOT_USED -1
#define I2S_WS_I2S0 25
#define I2S_SD_I2S0 33
#define I2S_SCK_I2S0 26
#define DMA_BUFFER_COUNT 4


int init_microphone() {
     i2s_driver_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = I2S_SAMPLE_RATE;
    i2s_config.bits_per_sample = (i2s_bits_per_sample_t)16;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_comm_format_t format = I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB;
    i2s_config.communication_format  =format;
    i2s_config.intr_alloc_flags = 0;
    i2s_config.dma_buf_count = DMA_BUFFER_COUNT;
    i2s_config.dma_buf_len = DMA_BUFFER_LEN;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;

    int result = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        return -1;
    }

    i2s_pin_config_t pin_config_one = {};
    pin_config_one.bck_io_num = I2S_SCK_I2S0;
    pin_config_one.ws_io_num = I2S_WS_I2S0;
    pin_config_one.data_out_num = NOT_USED;
    pin_config_one.data_in_num = I2S_SD_I2S0;

    result = i2s_set_pin(I2S_NUM_0, &pin_config_one);
    if (result != ESP_OK) {
        return -1;
    }

    return -1;
}



int getDataFromMicrophone(uint16_t data[], size_t bufferDepth) {

    size_t bytesRead = 0;
    esp_err_t result_I2S0 = i2s_read(I2S_NUM_0, data, sizeof(int16_t) * bufferDepth,
                                     &bytesRead, TICKS_TO_WAIT);
    if (result_I2S0 != ESP_OK || bytesRead != sizeof(int16_t) * bufferDepth) {
        return -1;
    }
    return 0;
}


void func(void)
{

}
