#include <WiFi.h>
#include <esp_now.h>

// Dirección MAC del receptor ESP32 (debes reemplazar con la dirección real)
//uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0xC4, 0x0D, 0xD7};
uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0xF2, 0x2E, 0xC0};

#define BOARD_ID 1
#define THRESHOLD 40

RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

typedef struct struct_message {
    int id;
    int relay;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery success");
    esp_deep_sleep_start();
  }
  else 
  {
    Serial.println("Delivery fail");
    esp_deep_sleep_start();
  }
}

void print_wakeup_reason() 
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) 
  {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void print_wakeup_touchpad() 
{
  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch (touchPin) 
  {
    case 0:  Serial.println("Touch detected on GPIO 4"); break;
    case 1:  Serial.println("Touch detected on GPIO 0"); break;
    case 2:  Serial.println("Touch detected on GPIO 2"); break;
    case 3:  Serial.println("Touch detected on GPIO 15"); break;
    case 4:  Serial.println("Touch detected on GPIO 13"); break;
    case 5:  Serial.println("Touch detected on GPIO 12"); break;
    case 6:  Serial.println("Touch detected on GPIO 14"); break;
    case 7:  Serial.println("Touch detected on GPIO 27"); break;
    case 8:  Serial.println("Touch detected on GPIO 33"); break;
    case 9:  Serial.println("Touch detected on GPIO 32"); break;
    default: Serial.println("Wakeup not by touchpad"); break;
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error adding peer");
    return;
  }

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32 and touchpad too
  print_wakeup_reason();
  print_wakeup_touchpad();

  touchSleepWakeUpEnable(T3, THRESHOLD);
  touchSleepWakeUpEnable(T7, THRESHOLD);

  if (bootCount > 1)
  {
    myData.id = BOARD_ID;
    myData.relay = 1;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }
  else
  {
    esp_deep_sleep_start();
  }
}

void loop() {
  /*if (aux == 0 && ready == 0) {
    myData.id = BOARD_ID;
    myData.relay = 1;

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  } 
  else if (ready == 1) 
  {
    if ((millis() - lastTime) > timerDelay) 
    {
      myData.id = BOARD_ID;
      if (aux == 0)
      {
        myData.relay = 1;
        aux = 1;
      }
      else
      {
        myData.relay = 0;
        aux = 0;
      }

      // Send message via ESP-NOW
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      lastTime = millis();
    }
  }*/
}
