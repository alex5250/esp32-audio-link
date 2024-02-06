#include <stdio.h>
#include "esp32-wifi-raw.h"
#include "freertos/FreeRTOS.h"

#include "esp_event.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"


#include "string.h"



uint8_t seq;
/*
 * This is the (currently unofficial) 802.11 raw frame TX API,
 * defined in esp32-wifi-lib's libnet80211.a/ieee80211_output.o
 *
 * This declaration is all you need for using esp_wifi_80211_tx in your own application.
 */
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);


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




/**
 * @brief Sends a string over ESP32's WiFi interface using custom beacon frames.
 *
 * @param data Array of strings to be sent.
 * @param mode Determines the type of packet template to be used.
 *             Mode 1 for management packets, mode 2 for data packets.
 * @cite  https://github.com/Jeija/esp32-80211-tx/blob/master/main/main.c
 */
void sendStringOverWiFi(char *data[],int mode) {

    uint8_t packet_header[51]; //header to ieee802.11 packet
	uint8_t line = 0; // current line only need to calc seqnum
    uint16_t seqnum[TOTAL_LINES] = { 0 }; // total line is zero
	uint8_t packet_buffer[200]; // buffer for packet
    switch (mode) // switch for different transmitting modes
    {
    case 1:
        memcpy(packet_header, managment_packet_template, sizeof(managment_packet_template)); // copy header for MGMT packet
        break; // exit switch
    case 2:
        memcpy(packet_header, data_packet_template, sizeof(data_packet_template));  // copy header for DATA packet
        break;// exit switch
    }
		memcpy(packet_buffer, packet_header, BEACON_SSID_OFFSET - 1); // copy the packet_headder to buffer
		packet_buffer[BEACON_SSID_OFFSET - 1] = strlen(data); // set size of the data requrement of ie802.11 standart
		memcpy(&packet_buffer[BEACON_SSID_OFFSET], data, strlen(data));
		memcpy(&packet_buffer[BEACON_SSID_OFFSET + strlen(data)], &packet_header[BEACON_SSID_OFFSET], sizeof(packet_header) - BEACON_SSID_OFFSET);

		// Last byte of source address / BSSID will be line number - emulate multiple APs broadcasting one song line each
		packet_buffer[SRCADDR_OFFSET + 5] = line;
		packet_buffer[BSSID_OFFSET + 5] = line;

		// Update sequence number
		packet_buffer[SEQNUM_OFFSET] = (seqnum[line] & 0x0f) << 4;
		packet_buffer[SEQNUM_OFFSET + 1] = (seqnum[line] & 0xff0) >> 4;
		seqnum[line]++;
		if (seqnum[line] > 0xfff)
			seqnum[line] = 0;

		esp_wifi_80211_tx(WIFI_IF_AP, packet_buffer, sizeof(packet_header) + strlen(data[line]), false);

		if (++line >= TOTAL_LINES)
			line = 0;

        vTaskDelay(1  / portTICK_PERIOD_MS);
}


	

void init_esp32_wifi_raw(void)
{
// Initialize NVS
	
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	// Init dummy AP to specify a channel and get WiFi hardware into
	// a mode where we can send the actual fake beacon frames.
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t ap_config = {
		.ap = {
			.ssid = "esp32-beaconspam",
			.ssid_len = 0,
			.password = "dummypassword",
			.channel = 6,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = 1,
			.max_connection = 4,
			.beacon_interval = 60000
		}
	};

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	//

}

