#include <esp_now.h>
#include <WiFi.h>

// control pins
#define IN_1  12
#define IN_2  14
#define IN_3  27
#define IN_4  26

// struct to receive data
typedef struct struct_message {
    uint8_t num;
} struct_message;
struct_message myData;

// too keep track of connection
int prevTime = 0;
int currTime = 0;

void goAhead(){ 
      Serial.println("Forward");
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
  }

void goBack(){ 
      Serial.println("Reverse");
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
  }

void goRight(){ 
      Serial.println("Right");
      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
  }

void goLeft(){
      Serial.println("Left");
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
  }

void stopRobot(){  
      Serial.println("Stopped");
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, LOW);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, LOW);
 }

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  currTime = millis();
  memcpy(&myData, incomingData, sizeof(myData));

  if (myData.num == 0) {
    stopRobot();
  }
  else if (myData.num == 1) {
    goAhead();
  }
  else if (myData.num == 2) {
    goBack();
  }
  else if (myData.num == 3) {
    goLeft();
  }
  else if (myData.num == 4) {
    goRight();
  }
}
 
void setup() {
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT); 

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if (prevTime == currTime) {
    stopRobot();
  }
  prevTime = currTime;
  delay(1000);
}
