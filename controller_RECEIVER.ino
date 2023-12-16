#include <esp_now.h>
#include <WiFi.h>

#include <Servo.h>

#define SERVO_PIN 14 // ESP32 pin GPIO26 connected to servo motor

// Motor
int motor1Pin1 = 27; 
int motor1Pin2 = 25; 
int enable1Pin = 32; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 15; // servo motor header file was using don't know how many pwm channels, so in its header file, I set max limit of pwm channels to 14.
const int resolution = 8;

Servo servoMotor;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int speed;
    bool direction;
    int rudderAngle;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  Serial.print("Speed: ");
  Serial.print(myData.speed);
  Serial.print("             Direction: ");
  Serial.print(myData.direction);
  Serial.print("             Rudder: ");
  Serial.println(myData.rudderAngle);

  if (myData.direction) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
  }

  
  ledcWrite(pwmChannel, myData.speed);

  servoMotor.write(myData.rudderAngle);

}

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  
  ledcWrite(pwmChannel, 0);

  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

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

}
