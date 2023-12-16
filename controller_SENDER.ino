#include <esp_now.h>
#include <WiFi.h>

#define THROTTLE  34
#define RUDDER  35

int throttleValue = 0;

// RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x41, 0x4B, 0x08};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int speed;
  bool direction;
  int rudderAngle;
} struct_message;// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
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
    // Serial.println("Failed to add peer");
    return;
  }
}

int getSpeed(int reading) {
  if (reading == 0)
    return 255;
  else if (reading < 4)
    return 230;
  else if (reading < 9)
    return 215;
  else if (reading < 13)
    return 200;
  else if (reading < 18)
    return 0;
  else if (reading < 22)
    return 200;
  else if (reading < 27)
    return 215;
  else if (reading < 31)
    return 230;
  else return 255;
}

int getRudder(int reading) {
  if (reading == 0)
    return 130;
  else if (reading < 4)
    return 120;
  else if (reading < 9)
    return 110;
  else if (reading < 13)
    return 100;
  else if (reading < 18)
    return 90;
  else if (reading < 22)
    return 80;
  else if (reading < 27)
    return 70;
  else if (reading < 31)
    return 60;
  else return 50;
}

void loop() {
  // read X and Y analog valueYs
  analogReadResolution(5);
  throttleValue = analogRead(THROTTLE);
  myData.speed = getSpeed(throttleValue);
  myData.direction = (throttleValue < 17);
  myData.rudderAngle = getRudder(analogRead(RUDDER));

  Serial.print("speed = ");
  Serial.print(myData.speed);
  Serial.print("        direction = ");
  Serial.print(myData.direction);
  Serial.print("        rudder = ");
  Serial.println(myData.rudderAngle);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
}
