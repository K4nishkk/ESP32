#include <esp_now.h>
#include <WiFi.h>

#define VRX_PIN  26 // X-axis analog input from joystick

int valueX = 0; // to store the X-axis value

// RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x41, 0x4B, 0x08};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int speed;
  bool direction;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // read X analog values
  valueX = (analogRead(VRX_PIN) >> 4);

  // Set values to send
  if (valueX >= 128) { // forward
    myData.speed = valueX - 128;
    myData.direction = true;
  }
  else { // reverse
    myData.speed = 128 - valueX;
    myData.direction = false;
  }
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
