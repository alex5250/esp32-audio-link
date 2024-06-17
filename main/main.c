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
/* Parameters of sending ESPNOW data. */
typedef struct {
  bool unicast;   // Send unicast ESPNOW data.
  bool broadcast; // Send broadcast ESPNOW data.
  uint8_t state;  // Indicate that if has received broadcast ESPNOW data or not.
  uint32_t magic; // Magic number which is used to determine which device to
                  // send unicast ESPNOW data.
  uint16_t count;  // Total count of unicast ESPNOW data to be sent.
  uint16_t delay;  // Delay between sending two ESPNOW data, unit: ms.
  int len;         // Length of ESPNOW data to be sent, unit: byte.
  uint8_t *buffer; // Buffer pointing to ESPNOW data.
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

static const char *TAG = "espnow_example";
#define ESPNOW_QUEUE_SIZE 6

#define IS_BROADCAST_ADDR(addr)                                                \
  (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF,
                                                            0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq[1024] = {0, 0};

static void example_espnow_deinit(example_espnow_send_param_t *send_param);
typedef enum {
  EXAMPLE_ESPNOW_SEND_CB,
  EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  uint8_t *data;
  int data_len;
} example_espnow_event_recv_cb_t;

typedef union {
  example_espnow_event_send_cb_t send_cb;
  example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to
 * ESPNOW task. */
typedef struct {
  example_espnow_event_id_t id;
  example_espnow_event_info_t info;
} example_espnow_event_t;

enum {
  EXAMPLE_ESPNOW_DATA_BROADCAST,
  EXAMPLE_ESPNOW_DATA_UNICAST,
  EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
  uint8_t type;  // Broadcast or unicast ESPNOW data.
  uint8_t state; // Indicate that if has received broadcast ESPNOW data or not.
  uint16_t seq_num; // Sequence number of ESPNOW data.
  uint16_t crc;     // CRC16 value of ESPNOW data.
  uint32_t magic;   // Magic number which is used to determine which device to
                  // send unicast ESPNOW data.
  uint8_t payload[0]; // Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

static QueueHandle_t s_example_espnow_queue;

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param) {
  example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

  assert(send_param->len >= sizeof(example_espnow_data_t));

  buf->type = IS_BROADCAST_ADDR(send_param->dest_mac)
                  ? EXAMPLE_ESPNOW_DATA_BROADCAST
                  : EXAMPLE_ESPNOW_DATA_UNICAST;
  buf->state = send_param->state;
  buf->seq_num = s_example_espnow_seq[buf->type]++;
  buf->crc = 0;
  buf->magic = send_param->magic;
  /* Fill all remaining bytes after the data with random values */
  esp_fill_random(buf->payload,
                  send_param->len - sizeof(example_espnow_data_t));
  buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

void init_nvs() {
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

/* WiFi should start before using ESPNOW */
static void init_wifi_esp_now(void) {
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
                                   esp_now_send_status_t status) {
  example_espnow_event_t evt;
  example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

  if (mac_addr == NULL) {
    ESP_LOGE(TAG, "Send cb arg error");
    return;
  }

  evt.id = EXAMPLE_ESPNOW_SEND_CB;
  memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  send_cb->status = status;
  if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
    ESP_LOGW(TAG, "Send send queue fail");
  }
}

void send_esp_now_packets(void *pvParameter) {
  example_espnow_event_t evt;
  uint8_t recv_state = 0;
  uint16_t recv_seq = 0;
  uint32_t recv_magic = 0;
  bool is_broadcast = false;
  int ret;

  vTaskDelay(5000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "Start sending broadcast data");
  while(true){
  /* Start sending broadcast ESPNOW data. */
  example_espnow_send_param_t *send_param =
      (example_espnow_send_param_t *)pvParameter;
  if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) !=
      ESP_OK) {
ESP_LOGW(TAG, "Send error");
    //example_espnow_deinit(send_param);
  }
  else {
    ESP_LOGI(TAG, "ok");
  }

  }
}
  void app_main() {
    example_espnow_send_param_t *send_param;

    init_nvs();

    s_example_espnow_queue =
        xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
      ESP_LOGE(TAG, "Create mutex fail");
    }
    init_wifi_esp_now();
    init_esp_now();
   ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        //esp_now_deinit();
        //return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

     /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        //esp_now_deinit();
        //return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        //esp_now_deinit();
        //return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(send_esp_now_packets, "example_espnow_task", 4096, send_param, 4, NULL);
    // xTaskCreate(send_esp_now_packets, "example_espnow_task", 2048,
    // send_param, 4, NULL);
  }
