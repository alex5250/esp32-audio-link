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
static const char *TAG = "esp-audio-link";

#define VAD_SAMPLE_RATE_HZ 16000
#define VAD_FRAME_LENGTH_MS 10
#define VAD_BUFFER_LENGTH (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
#define FRAME_SIZE 160
#define PACKAGE_SIZE 200

/* Parameters of sending ESPNOW data. */
typedef struct
{
    bool unicast;                       // Send unicast ESPNOW data.
    bool broadcast;                     // Send broadcast ESPNOW data.
    uint8_t state;                      // Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                     // Magic number which is used to determine which device to
                                        // send unicast ESPNOW data.
    uint16_t count;                     // Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                     // Delay between sending two ESPNOW data, unit: ms.
    int len;                            // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                    // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
} example_espnow_send_param_t;
/* ESPNOW can work in both station and softap mode. It is configured in
 * menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#endif
#define ESPNOW_MAXDELAY 512
#define ESPNOW_QUEUE_SIZE 6

#define IS_BROADCAST_ADDR(addr) \
    (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                            0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq[1024] = {0, 0};

static void example_espnow_deinit(example_espnow_send_param_t *send_param);
typedef enum
{
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

typedef union
{
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to
 * ESPNOW task. */
typedef struct
{
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

enum
{
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct
{
    uint8_t type;       // Broadcast or unicast ESPNOW data.
    uint8_t state;      // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;   // Sequence number of ESPNOW data.
    uint16_t crc;       // CRC16 value of ESPNOW data.
    uint32_t magic;     // Magic number which is used to determine which device to
                        // send unicast ESPNOW data.
    uint8_t payload[40]; // Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

static QueueHandle_t s_example_espnow_queue;

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param, uint8_t *buffer_to_send) {
    ESP_LOGI(TAG, "Preparing ESPNOW data");

    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    ESP_LOGI(TAG, "Buffer casted to example_espnow_data_t");

    // Ensure buffer si:ze is sufficient
    assert(send_param->len >= sizeof(example_espnow_data_t));
    ESP_LOGI(TAG, "Buffer length is sufficient");

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    ESP_LOGI(TAG, "Set type to %d", buf->type);

    buf->state = send_param->state;
    ESP_LOGI(TAG, "Set state to %d", buf->state);

    buf->seq_num = s_example_espnow_seq[buf->type]++;
    ESP_LOGI(TAG, "Set sequence number to %d", buf->seq_num);

    buf->crc = 0;
    ESP_LOGI(TAG, "Initialized CRC to 0");

    buf->magic = send_param->magic;
    ESP_LOGI(TAG, "Set magic number to %u", buf->magic);

    // Calculate the maximum payload length
    int max_payload_len = send_param->len - sizeof(example_espnow_data_t);
    ESP_LOGI(TAG, "Calculated maximum payload length: %d bytes", max_payload_len);

    // Initialize payload with zeros
    memset(buf->payload, 0, max_payload_len);
    ESP_LOGI(TAG, "Payload initialized with zeros");

    // Copy buffer_to_send to buf->payload
    memcpy(buf->payload, buffer_to_send, max_payload_len);
    ESP_LOGI(TAG, "Copied data to payload");

    // Calculate the CRC of the entire buffer (including the payload)
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
    ESP_LOGI(TAG, "Calculated CRC: %u", buf->crc);
}



void init_nvs()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* WiFi should start before using ESPNOW */
static void init_wifi_esp_now(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, 0));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                            WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

void init_esp_now() { ESP_ERROR_CHECK(esp_now_init()); }

static void example_espnow_send_cb(const uint8_t *mac_addr,
                                   esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;

    if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

void send_esp_now_packets(example_espnow_send_param_t *send_param)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    //vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

        /* Start sending broadcast ESPNOW data. */
        int status = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
         ESP_LOGI(TAG, "esp status %d",status);
        if (status !=
            ESP_OK)
        {
            ESP_LOGW(TAG, "Send error");
            // example_espnow_deinit(send_param);
        }
        else
        {
            ESP_LOGI(TAG, "ok");
        }
    }


bool continue_recording = false;
void init_sd_card()
{
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
    switch ((int)evt->data)
    {
    case INPUT_KEY_USER_ID_REC:
        ESP_LOGI(TAG, "[ * ] [Rec] KEY %s", key_types[evt->type]);
        continue_recording = !continue_recording;
        ESP_LOGI(TAG, "updated continue recording state");
        break;

    default:
        ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
        break;
    }
    return ESP_OK;
}

void init_adc_buttons()
{
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

void speex_encode_decode_task(audio_element_handle_t raw_read, example_espnow_send_param_t *send_param)
{
    ESP_LOGI(TAG, "================ Start encode =================");

    short in[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized input buffer");

    float input[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized float input buffer");

    char cbits[200];
    ESP_LOGI(TAG, "Initialized cbits buffer");

    int nbBytes;
    ESP_LOGI(TAG, "Initialized nbBytes variable");
    int16_t out[FRAME_SIZE];
    char buf[PACKAGE_SIZE];

    // Encoder state
    void *encoder_state = speex_encoder_init(&speex_nb_mode);
    if (!encoder_state)
    {
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
    if (!decoder_state)
    {
        ESP_LOGE(TAG, "Failed to initialize decoder");
        speex_encoder_destroy(encoder_state);
        return;
    }
    ESP_LOGI(TAG, "Decoder initialized");

    // Set perceptual enhancement on
    speex_decoder_ctl(decoder_state, SPEEX_SET_ENH, &tmp);
    ESP_LOGI(TAG, "Decoder enhancement set");

    // Bit-stream structures
    SpeexBits enc_bits, dec_bits;
    speex_bits_init(&enc_bits);
    ESP_LOGI(TAG, "Encoder bits initialized");

    speex_bits_init(&dec_bits);
    ESP_LOGI(TAG, "Decoder bits initialized");

    // Open files


    // Encoding and Decoding loop
    while (1)
    {
        // Read raw audio data
        raw_stream_read(raw_read, (char *)in, FRAME_SIZE * sizeof(short));
        ESP_LOGI(TAG, "Read raw audio data");

        // Check for end of input (optional based on raw_stream_read behavior)
        if (feof(stdin))
            break;

        ESP_LOGI(TAG, "Converted input to float for Speex encoder");

        // Encode audio data
        speex_bits_reset(&enc_bits);
        ESP_LOGI(TAG, "Reset encoder bits");

        speex_encode(encoder_state, input, &enc_bits);
        ESP_LOGI(TAG, "Encoded audio data");

        // Write encoded bits to buffer
        nbBytes = speex_bits_write(&enc_bits, in, sizeof(in));
        ESP_LOGI(TAG, "Wrote encoded bits to buffer: %d bytes", nbBytes);
 ESP_LOGI(TAG, "Initializing send_param structure");
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    
    ESP_LOGI(TAG, "Setting unicast to true and broadcast to true");
    send_param->unicast = true;
    send_param->broadcast = true;
    
    ESP_LOGI(TAG, "Setting state to 0");
    send_param->state = 0;
    
    ESP_LOGI(TAG, "Generating random magic number");
    send_param->magic = esp_random();
    
    ESP_LOGI(TAG, "Setting count to %d", CONFIG_ESPNOW_SEND_COUNT);
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    
    ESP_LOGI(TAG, "Setting delay to %d", CONFIG_ESPNOW_SEND_DELAY);
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    
    ESP_LOGI(TAG, "Setting length to %d", CONFIG_ESPNOW_SEND_LEN);
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    
    ESP_LOGI(TAG, "Allocating buffer of size %d bytes", CONFIG_ESPNOW_SEND_LEN);
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer failed");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        // esp_now_deinit();
        // return ESP_FAIL;
        return;  // Ensure you exit the function to prevent further issues.
    }
    
    ESP_LOGI(TAG, "Copying destination MAC address");
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    
    ESP_LOGI(TAG, "Preparing ESPNOW data");
    example_espnow_data_prepare(send_param, nbBytes);
    
    ESP_LOGI(TAG, "Sending ESPNOW packets");
    send_esp_now_packets(send_param);

    // Optional: Break condition or sleep to prevent infinite looping
    if (continue_recording)
    {
        ESP_LOGI(TAG, "Recording stopped, breaking loop");
        break;
    }
    }
   
   
}

void start_audio_link(void *pvParameters )
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
    rsp_cfg.dest_rate = VAD_SAMPLE_RATE_HZ / 2;
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

    example_espnow_send_param_t *send_param;

    init_nvs();

    s_example_espnow_queue =
        xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
    }
    init_wifi_esp_now();
    init_esp_now();
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        // esp_now_deinit();
        // return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        // esp_now_deinit();
        // return ESP_FAIL;
    }

    // send_data();

    speex_encode_decode_task(raw_read, send_param);
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

/*
 */