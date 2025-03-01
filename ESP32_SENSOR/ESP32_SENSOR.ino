#include <WiFi.h>
#include <esp_now.h>
#include "driver/rtc_io.h"

//#define DEBUG_MODE

// Dirección MAC de la cámara:
//uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0xC4, 0x0D, 0xD7};
//uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0xF2, 0x2E, 0xC0};
uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xA1, 0x97, 0x08};

#define BOARD_ID 1

#define WAKEUP_GPIO GPIO_NUM_33     // Only RTC IO are allowed - ESP32 Pin example
RTC_DATA_ATTR int bootCount = 0;
int attempts = 0;

typedef struct struct_message {
    int id;
    int state;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef DEBUG_MODE
  Serial.print("\r\nLast Packet Send Status: ");
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    attempts = 0;
#ifdef DEBUG_MODE
    Serial.println("Delivery success");
#endif
  }
  else 
  {
    if (attempts < 5)
    {
      attempts++;
#ifdef DEBUG_MODE
      Serial.println("Delivery fail, trying again");
#endif
      delay(200);
      myData.id = BOARD_ID;
      myData.state = 1;
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    }
    else
    {
      attempts = 0;
    }
  }
}

void initComms()
{
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
#ifdef DEBUG_MODE
    Serial.println("Error initializing ESP-NOW");
#endif    
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA; 

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  #ifdef DEBUG_MODE
    Serial.println("Error adding peer");
  #endif
    return;
  }
}

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
  delay(1000);
#endif

  initComms();
  if (bootCount > 0)
  {
    myData.id = BOARD_ID;
    myData.state = 1;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }

  ++bootCount;
#ifdef DEBUG_MODE
  Serial.println("Boot number: " + String(bootCount));
#endif

  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);  //1 = High, 0 = Low

  delay(1000);
#ifdef DEBUG_MODE
  Serial.println("Going to sleep now");
#endif
  esp_deep_sleep_start();
}

void loop() {

}
