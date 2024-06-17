/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  wifi_pkt_rx_ctrl_t *val = esp_now_info->rx_ctrl;
  uint8_t *address = esp_now_info->src_addr;
  uint8_t array[250];

  memcpy(&array, incomingData, sizeof(array));
  Serial.print("RSSI");
  Serial.print(val->rssi);
  Serial.print(" SRC: ");
  Serial.print(*address, HEX);
  Serial.print(" DATA: ");
  for (int i = 0; i < len - 1; i++) {
    Serial.print(array[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

}




void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
}
