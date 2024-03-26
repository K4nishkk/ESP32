#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    uint8_t num;
} struct_message;

// Create a struct_message called myData
struct_message myData;

int prevTime = 0;
int currTime = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  currTime = millis();
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println(myData.num);
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
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if (prevTime == currTime) {
    Serial.println("Connection stopped");
  }
  prevTime = currTime;
  delay(1000);
}
