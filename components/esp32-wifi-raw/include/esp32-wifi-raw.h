#ifndef ESP32_WIFI_RAW_H
#define ESP32_WIFI_RAW_H

#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_system.h"


// Define any constants used in the code
#define BEACON_SSID_OFFSET 38
#define SRCADDR_OFFSET 10
#define BSSID_OFFSET 16
#define SEQNUM_OFFSET 22
#define TOTAL_LINES 10

// Declaration of the unofficial 802.11 raw frame TX API
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

// Template arrays for management and data packets
//extern uint8_t managment_packet_template[];
//extern uint8_t data_packet_template[];

// Function prototypes
void ieee_reciver_init(void);
const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
void wifi_sniffer_set_channel(uint8_t channel);
void recivePacket(int size, int buffer, int channel);
void sendTelemetryOverIEEE(void);
void sendStringOverIEEE(char *data[]);
void init_esp32_wifi_raw(void);
//esp_err_t event_handler(void *ctx, system_event_t *event);
void run_on_event(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);

#endif // ESP32_WIFI_RAW_H
