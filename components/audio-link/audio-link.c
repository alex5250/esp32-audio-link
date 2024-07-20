/* Example of Voice Activity Detection (VAD)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "board.h"
#include "audio_common.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "filter_resample.h"
#include "esp_vad.h"
#include <speex/speex.h>
#include "audio_idf_version.h"
#include <esp_vfs_fat.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include "esp_log.h"
#include "board.h"
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"
static const char *TAG = "esp-audio-link";

#define VAD_SAMPLE_RATE_HZ 16000
#define VAD_FRAME_LENGTH_MS 30
#define VAD_BUFFER_LENGTH (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
#define FRAME_SIZE   3840
#define PACKAGE_SIZE 7680
bool continue_recording = false;
void init_sd_card() {
 ESP_LOGI(TAG, "[1.0] Initialize peripherals management");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[1.1] Initialize and start peripherals");
    audio_board_key_init(set);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);


}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
             ESP_LOGD(TAG, "[ * ] input key id is %d, %d", (int)evt->data, evt->type);
    const char *key_types[INPUT_KEY_SERVICE_ACTION_PRESS_RELEASE + 1] = {"UNKNOWN", "CLICKED", "CLICK RELEASED", "PRESSED", "PRESS RELEASED"};
    switch ((int)evt->data) {
        case INPUT_KEY_USER_ID_REC:
            ESP_LOGI(TAG, "[ * ] [Rec] KEY %s", key_types[evt->type]);
            continue_recording = !continue_recording;
            ESP_LOGI(TAG,"updated continue recording state");
            break;

          default:
            ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
            break;
}
return ESP_OK;
}


void init_adc_buttons() {
        ESP_LOGI(TAG, "[ 1 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[ 2 ] Initialize Button peripheral with board init");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 3 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, NULL);

    ESP_LOGW(TAG, "[ 4 ] Waiting for a button to be pressed ...");
}

void speex_encode_decode_task(audio_element_handle_t  raw_read) {
    ESP_LOGI(TAG, "================ Start encode =================");

    short in[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized input buffer");

    float input[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized float input buffer");

    char cbits[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized cbits buffer");

    int nbBytes;
    ESP_LOGI(TAG, "Initialized nbBytes variable");

    // Encoder state
    void *encoder_state = speex_encoder_init(&speex_nb_mode);
    if (!encoder_state) {
        ESP_LOGE(TAG, "Failed to initialize encoder");
        return;
    }
    ESP_LOGI(TAG, "Encoder initialized");

    // Set the quality to 8 (15 kbps)
    int tmp = 8;
    speex_encoder_ctl(encoder_state, SPEEX_SET_QUALITY, &tmp);
    ESP_LOGI(TAG, "Encoder quality set to 8");

    // Decoder state
    void *decoder_state = speex_decoder_init(&speex_nb_mode);
    if (!decoder_state) {
        ESP_LOGE(TAG, "Failed to initialize decoder");
        speex_encoder_destroy(encoder_state);
        return;
    }
    ESP_LOGI(TAG, "Decoder initialized");

    // Set perceptual enhancement on
    //speex_decoder_ctl(decoder_state, SPEEX_SET_ENH, 1);
    ESP_LOGI(TAG, "Decoder enhancement set");

    // Bit-stream structures
    SpeexBits enc_bits, dec_bits;
    speex_bits_init(&enc_bits);
    ESP_LOGI(TAG, "Encoder bits initialized");

    speex_bits_init(&dec_bits);
    ESP_LOGI(TAG, "Decoder bits initialized");

    // Open files
    FILE *fout_enc = fopen("/sdcard/speex.spx", "w");
    if (!fout_enc) {
        ESP_LOGE(TAG, "Could not open /sdcard/speex.spx for writing");
        speex_encoder_destroy(encoder_state);
        speex_decoder_destroy(decoder_state);
        return;
    }
    ESP_LOGI(TAG, "Opened /sdcard/speex.spx for writing");

    FILE *fout_dec = fopen("/sdcard/drawpcm.wav", "w");
    if (!fout_dec) {
        ESP_LOGE(TAG, "Could not open /sdcard/decoded_rawpcm.wav for writing");
        fclose(fout_enc);
        speex_encoder_destroy(encoder_state);
        speex_decoder_destroy(decoder_state);
        return;

    }
     FILE *fout_raw = fopen("/sdcard/raw.wav", "w");
    if (!fout_raw) {
        ESP_LOGE(TAG, "Could not open /sdcard/raw.wav for writing");
        fclose(fout_raw);
        
    }
    ESP_LOGI(TAG, "Opened /sdcard/raw.wav for writing");

    ESP_LOGI(TAG, "================ Encoding and Decoding loop start =================");

    // Encoding and Decoding loop
    while (1) {
        // Read raw audio data
        raw_stream_read(raw_read, (char *)in, FRAME_SIZE * sizeof(short));

            if (fwrite(in, sizeof(int16_t), FRAME_SIZE, fout_raw) != FRAME_SIZE) {
            ESP_LOGE(TAG, "Failed to raw decoded data to file");
            break;
        }

        
        ESP_LOGI(TAG, "Read raw audio data");

        // Check for end of input (optional based on raw_stream_read behavior)
        if (feof(stdin)) break;

        // Convert to float for Speex encoder
        for (int i = 0; i < FRAME_SIZE; i++) {
            input[i] = in[i];
        }
        ESP_LOGI(TAG, "Converted input to float for Speex encoder");

        // Encode audio data
        speex_bits_reset(&enc_bits);
        ESP_LOGI(TAG, "Reset encoder bits");

        speex_encode(encoder_state, in, &enc_bits);
        ESP_LOGI(TAG, "Encoded audio data");

        // Write encoded bits to buffer
        nbBytes = speex_bits_write(&enc_bits, cbits, sizeof(cbits));
        ESP_LOGI(TAG, "Wrote encoded bits to buffer: %d bytes", nbBytes);

        if (fwrite(cbits, sizeof(int16_t), nbBytes, fout_enc) != nbBytes) {
            ESP_LOGE(TAG, "Failed to write encoded data to file");
            break;
        }
        ESP_LOGI(TAG, "Wrote encoded data to file");

        // Decode the same data immediately
        //speex_bits_reset(&dec_bits);
        ESP_LOGI(TAG, "Reset decoder bits");

        speex_bits_read_from(&dec_bits, cbits, nbBytes);
        ESP_LOGI(TAG, "Read encoded bits into decoder bits");

        int16_t out[FRAME_SIZE];
        if (speex_decode_int(decoder_state, &dec_bits, out) != 0) {
            ESP_LOGE(TAG, "Failed to decode data");
            break;
        }
        ESP_LOGI(TAG, "Decoded audio data");


     

        // Write decoded data to file
        if (fwrite(out, sizeof(int16_t), FRAME_SIZE, fout_dec) != FRAME_SIZE) {
            ESP_LOGE(TAG, "Failed to write decoded data to file");
            break;
        }
        ESP_LOGI(TAG, "Wrote decoded data to file");
        
        // Optional: Break condition or sleep to prevent infinite looping
        if (continue_recording) break;
    }

    ESP_LOGI(TAG, "================ Encoding and Decoding loop end =================");

    // Destroy encoder and decoder states
    speex_encoder_destroy(encoder_state);
    ESP_LOGI(TAG, "Destroyed encoder state");

    speex_decoder_destroy(decoder_state);
    ESP_LOGI(TAG, "Destroyed decoder state");

    // Destroy bit-stream structures
    speex_bits_destroy(&enc_bits);
    ESP_LOGI(TAG, "Destroyed encoder bits");

    speex_bits_destroy(&dec_bits);
    ESP_LOGI(TAG, "Destroyed decoder bits");

    // Close files
    fclose(fout_enc);
    ESP_LOGI(TAG, "Closed encoded file");

    fclose(fout_dec);
    ESP_LOGI(TAG, "Closed decoded file");

    fclose(fout_raw);

    ESP_LOGI(TAG, "Encode and decode done");
}
    void start_audio_link()
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader, filter, raw_read;

    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline for recording");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(CODEC_ADC_I2S_PORT, 48000, 16, AUDIO_STREAM_READER);
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.2] Create filter to resample audio data");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 16000;
    rsp_cfg.src_ch = 1;
    rsp_cfg.dest_rate = VAD_SAMPLE_RATE_HZ/2;
    rsp_cfg.dest_ch = 1;
    filter = rsp_filter_init(&rsp_cfg);

    ESP_LOGI(TAG, "[2.3] Create raw to receive data");
    raw_stream_cfg_t raw_cfg = {
        .out_rb_size = 8 * 1024,
        .type = AUDIO_STREAM_READER,
    };
    raw_read = raw_stream_init(&raw_cfg);

    ESP_LOGI(TAG, "[ 3 ] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, filter, "filter");
    audio_pipeline_register(pipeline, raw_read, "raw");

    ESP_LOGI(TAG, "[ 4 ] Link elements together [codec_chip]-->i2s_stream-->filter-->raw-->[VAD]");
    const char *link_tag[3] = {"i2s", "filter", "raw"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);



    ESP_LOGI(TAG, "[ 6 ] Inint speex ");

  

     speex_encode_decode_task(raw_read);
    ESP_LOGI(TAG, "[ 8 ] Stop audio_pipeline and release all resources");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);






    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, filter);
    audio_pipeline_unregister(pipeline, raw_read);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(filter);
    audio_element_deinit(raw_read);

   // speex_decode_task();

}



