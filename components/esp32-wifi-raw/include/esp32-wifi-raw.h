


#ifndef ESP32_WIFI_RAW_H
#define ESP32_WIFI_RAW_H

#include "esp_err.h"
#include "esp_wifi_types.h"

// Macro definitions
#define BEACON_SSID_OFFSET 38
#define SRCADDR_OFFSET 10
#define BSSID_OFFSET 16
#define SEQNUM_OFFSET 22
#define TOTAL_LINES 10




// Function prototypes
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
void sendStringOverWiFi(char *data[], int mode);
void init_esp32_wifi_raw(void);


#endif // ESP32_WIFI_RAW_H