#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define LED_GPIO_PIN 5
#define WIFI_CHANNEL_SWITCH_INTERVAL (500)
#define WIFI_CHANNEL_MAX (13)

uint8_t level = 0, channel = 1;

static wifi_country_t wifi_country = { .cc = "CN", .schan = 1, .nchan = 13 };  //Most recent esp32 library struct

typedef struct {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void wifi_sniffer_init(void) {
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type) {
  switch (type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    default:
    case WIFI_PKT_MISC: return "MISC";
  }
}
void printDataInDecimal(const uint8_t *data) {
  //Serial.print("Data in Decimal: ");
  for (int i = 0; data[i] != '\0'; i++) {
    //Serial.print(data[i]);
    //Serial.print(" ");
  }
  //Serial.println();
}

String intArrayToString(int arr[], int length) {
  String result = "";

  for (int i = 0; i < length; i++) {
    // Convert each integer to a string and concatenate it to the result
    result += String(arr[i]);

    // Add a delimiter (e.g., a comma) if it's not the last element
    if (i < length - 1) {
      result += ",";
    }
  }

  return result;
}
void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type) {
  //if (type != WIFI_PKT_MGMT)
  //return;



  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  if (ppkt->rx_ctrl.rssi > -50) {
    if (type == WIFI_PKT_DATA) {
      //Serial.print("Data sent size: ");
      int size = ppkt->rx_ctrl.sig_len;
      //Serial.print("Size: ");
      //Serial.print(size);
      // Serial.print("  Data: ");

      // Access the payload data
      const uint8_t *payload_data = ipkt->payload;

      int recived_data[size];

      // Process the payload data as needed
      for (int i = 0; i < size; i++) {
        int value = (int)payload_data[i];
        if (value > 31 && value < 126) {
          char out_char = (char) value;
          //recived_data[i] = ;
          Serial.print(out_char);
        }
      }

      Serial.println();
      //Serial.print("To string: ");

      //intArrayToString(recived_data, size);
    }
  }
}



// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 5 as an output.
  Serial.begin(115200);
  delay(10);
  wifi_sniffer_init();
  pinMode(LED_GPIO_PIN, OUTPUT);
  channel = 6;
}

// the loop function runs over and over again forever
void loop() {
  //Serial.print("inside loop");
  delay(1000);  // wait for a second

  if (digitalRead(LED_GPIO_PIN) == LOW)
    digitalWrite(LED_GPIO_PIN, HIGH);
  else
    digitalWrite(LED_GPIO_PIN, LOW);
  vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
  wifi_sniffer_set_channel(channel);
  //channel = (channel % WIFI_CHANNEL_MAX) + 1;
}
