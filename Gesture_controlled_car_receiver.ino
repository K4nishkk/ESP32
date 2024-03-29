// ******************************************** HEADER FILES *************************************
#include <esp_now.h>
#include <WiFi.h>

// ************************************** VARIABLES AND FUNCTIONS ********************************************

// ======================== MOTOR CONTROL ============================
// control pins
#define ENA 33
#define ENB 32
#define IN_1 12
#define IN_2 14
#define IN_3 27
#define IN_4 26

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

void goAhead() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);

  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
}

void goBack() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
}

void goRight() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
}

void goLeft() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);

  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
}

void stopRobot() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
}

// ============================= ESP-NOW ===================================
typedef struct struct_message {
  uint8_t dir;
  uint8_t speed;
} struct_message;
struct_message myData;

// too keep track of connection
int prevTime = 0;
int currTime = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  currTime = millis();
  memcpy(&myData, incomingData, sizeof(myData));

  int dutyCycle = map(myData.speed, 1, 6, 180, 255);
  ledcWrite(pwmChannel, dutyCycle);

  if (myData.dir == 0) {
    stopRobot();
  } else if (myData.dir == 1) {
    goAhead();
  } else if (myData.dir == 2) {
    goBack();
  } else if (myData.dir == 3) {
    goLeft();
  } else if (myData.dir == 4) {
    goRight();
  }

  Serial.print(myData.dir);
  Serial.print("\t");
  Serial.println(myData.speed);
}

// ************************************************ SETUP ***********************************************

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENA, pwmChannel);
  ledcAttachPin(ENB, pwmChannel);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// ************************************************ LOOP *************************************************

void loop() {
  if (prevTime == currTime) {
    Serial.println("Not Connected");
    stopRobot();
  }
  prevTime = currTime;
  delay(1000);
}
