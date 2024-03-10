#include "esp32-wifi-raw.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "string.h"
#include <stdio.h>
#include "tcpip_adapter.h"
#include "esp_system.h"

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
//static void wifi_sniffer_set_channel(uint8_t channel);
//static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);


uint8_t seq;
/*
 * This is the (currently unofficial) 802.11 raw frame TX API,
 * defined in esp32-wifi-lib's libnet80211.a/ieee80211_output.o
 *
 * This declaration is all you need for using esp_wifi_80211_tx in your own
 * application.
 */
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len,
                            bool en_sys_seq);

uint8_t managment_packet_template[] = {
  0x80, 0x00,							// 0-1: Frame Control
	0x00, 0x00,							// 2-3: Duration
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				// 10-15: Source address
	0x00, 0x00,							// 22-23: Sequence / fragment number
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,			// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
	0x64, 0x00,							// 32-33: Beacon interval
	0x31, 0x04,							// 34-35: Capability info
	0x00, 0x00, /* FILL CONTENT HERE */				// 36-38: SSID parameter set, 0x00:length:content
	0x01, 0x08, 0x82, 0x84,	0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,	// 39-48: Supported rates
	0x03, 0x01, 0x01,						// 49-51: DS Parameter set, current channel 1 (= 0x01),
	0x05, 0x04, 0x01, 0x02, 0x00, 0x00,				// 52-57: Traffic Indication Map
	
};
uint8_t data_packet_template[] = {
	
  0x08, 0x01,							// 0-1: Frame Control
	0x00, 0x00,							// 2-3: Duration
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				// 10-15: Source address
	0x00, 0x00,							// 22-23: Sequence / fragment number
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,			// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
	0x64, 0x00,							// 32-33: Beacon interval
	0x31, 0x04,							// 34-35: Capability info
	0x00, 0x00, /* FILL CONTENT HERE */				// 36-38: SSID parameter set, 0x00:length:content
	0x01, 0x08, 0x82, 0x84,	0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,	// 39-48: Supported rates
	0x03, 0x01, 0x01,						// 49-51: DS Parameter set, current channel 1 (= 0x01),
	0x05, 0x04, 0x01, 0x02, 0x00, 0x00,				// 52-57: Traffic Indication Map
	
};
#define BEACON_SSID_OFFSET 38
#define SRCADDR_OFFSET 10
#define BSSID_OFFSET 16
#define SEQNUM_OFFSET 22
#define TOTAL_LINES 10


extern int decode_rs_8(uint8_t *data, int *eras_pos, int no_eras, int pad);

static wifi_country_t wifi_country = { .cc = "CN", .schan = 1, .nchan = 13 };  //Most recent esp32 library struct

void run_on_event(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    return ESP_OK;
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);



  


const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type) {
  switch (type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    default:
    case WIFI_PKT_MISC: return "MISC";
  }
}

void wifi_sniffer_set_channel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

void recivePacket(int size,int buffer,int channel) {
   wifi_sniffer_set_channel(channel);


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
uint8_t* nonConstPointer = (uint8_t*)payload_data;
      decode_rs_8(&nonConstPointer[size],0,0,223-64);

      int recived_data[size];
      char buf[size];
      // Process the payload data as needed
      for (int i = 0; i < size; i++) {
        int value = (int)payload_data[i];
        if (value > 31 && value < 126) {
          char out_char = (char) value;
         
          //recived_data[i] = ;
          //Serial.print(out_char);
        }
         printf("%s",buf);
      }

      //Serial.println();
      //Serial.print("To string: ");

      //intArrayToString(recived_data, size);
    }
  }
}




/**
 * @brief Sends a string over ESP32's WiFi interface using custom beacon frames.
 *
 * @param data Array of strings to be sent.
 * @param mode Determines the type of packet template to be used.
 *             Mode 1 for management packets, mode 2 for data packets.
 * @cite  https://github.com/Jeija/esp32-80211-tx/blob/master/main/main.c
 */
void sendStringOverIEEE(char *data[]) {

  uint8_t packet_header[51];          // header to ieee802.11 packet
  uint8_t line = 0;                   // current line only need to calc seqnum
  uint16_t seqnum[TOTAL_LINES] = {0}; // total line is zero
  uint8_t packet_buffer[200];         // buffer for packet
 
  memcpy(packet_header, data_packet_template,
           sizeof(data_packet_template));
  memcpy(packet_buffer, packet_header,
         BEACON_SSID_OFFSET - 1); // copy the packet_headder to buffer
  packet_buffer[BEACON_SSID_OFFSET - 1] =
      strlen(data); // set size of the data requrement of ie802.11 standart
  memcpy(&packet_buffer[BEACON_SSID_OFFSET], data, strlen(data));
  memcpy(&packet_buffer[BEACON_SSID_OFFSET + strlen(data)],
         &packet_header[BEACON_SSID_OFFSET],
         sizeof(packet_header) - BEACON_SSID_OFFSET);

  // Last byte of source address / BSSID will be line number - emulate multiple
  // APs broadcasting one song line each
  packet_buffer[SRCADDR_OFFSET + 5] = line;
  packet_buffer[BSSID_OFFSET + 5] = line;

  // Update sequence number
  packet_buffer[SEQNUM_OFFSET] = (seqnum[line] & 0x0f) << 4;
  packet_buffer[SEQNUM_OFFSET + 1] = (seqnum[line] & 0xff0) >> 4;
  seqnum[line]++;
  if (seqnum[line] > 0xfff)
    seqnum[line] = 0;

  esp_wifi_80211_tx(WIFI_IF_AP, packet_buffer,
                    sizeof(packet_header) + strlen(data[line]), false);

  if (++line >= TOTAL_LINES)
    line = 0;

  vTaskDelay(1 / portTICK_PERIOD_MS);
}
void init_esp32_wifi_raw(void) {
  // Initialize NVS
//tcpip_adapter_init();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  // Init dummy AP to specify a channel and get WiFi hardware into
  // a mode where we can send the actual fake beacon frames.
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  wifi_config_t ap_config = {.ap = {.ssid = "esp32-beaconspam",
                                    .ssid_len = 0,
                                    .password = "dummypassword",
                                    .channel = 6,
                                    .authmode = WIFI_AUTH_WPA2_PSK,
                                    .ssid_hidden = 1,
                                    .max_connection = 4,
                                    .beacon_interval = 60000}};

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  
   esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}
