#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>

// Network and Firebase credentials
#define WIFI_SSID "Kanishk"
#define WIFI_PASSWORD "passcode"
#define Web_API_KEY "AIzaSyAZaj3uWazzyg0CwGI_SpV4kAxxO0FwWp0"
#define DATABASE_URL "https://majorproject-285f3-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "kanishkk462462@gmail.com"
#define USER_PASS "0987654321"

// Firebase setup
UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

// Function prototype
void processData(AsyncResult &aResult);

// Relay setup
#define RELAY_PIN 18
bool relayStatus = true;
bool isGettingRelayStatus = false;  // Flag to prevent overlapping GETs

// ADC
Adafruit_ADS1115 ads;
const float adcToVolt = 0.0001875f;
int16_t peakADC = 0;

// Timing
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;

unsigned long lastRelayCheck = 0;
const unsigned long relayCheckInterval = 5000;

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWi-Fi Connected");

  ssl_client.setInsecure();
  ssl_client.setHandshakeTimeout(5);

  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Initially ON

  Wire.begin();
  Wire.setClock(400000);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115!");
  }
  ads.setDataRate(RATE_ADS1115_860SPS);
}

void loop() {
  app.loop();

  // Wi-Fi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(500);
    return;
  }

  if (app.ready() && relayStatus) {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    if (adc0 > peakADC) peakADC = adc0;

    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= sendInterval) {
      lastSendTime = currentTime;

      float peakVoltage = (float)peakADC * adcToVolt;
      Database.set<float>(aClient, "/data/adcVoltage", peakVoltage, processData, "Send_ADC");

      if (peakVoltage > 2.6) {
        relayStatus = false;
        digitalWrite(RELAY_PIN, HIGH);  // Trip relay
        Database.set<bool>(aClient, "/data/relayStatus", relayStatus, processData, "Trip_Relay");
          // Set each field for fault object
          Database.set<String>(aClient, "/faults/fault/id", "fault123", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/severity", "high", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/description", "Power outage near transformer", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/otp", String(random(0, 1000000)), processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/reportedAt", "2025-05-26T12:00:00Z", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/status", "pending", processData, "RTDB_send_fault");

          // Assuming location is an object, you can serialize it as a string or store as individual fields:
          Database.set<String>(aClient, "/faults/fault/location/address", "123 Main St", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/location/city", "Springfield", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/location/coordinates/lat", "12.3456", processData, "RTDB_send_fault");
          Database.set<String>(aClient, "/faults/fault/location/coordinates/lng", "78.9012", processData, "RTDB_send_fault");

      }

      peakADC = 0;
    }
  }

  // Firebase check to re-enable relay
  if (!relayStatus && !isGettingRelayStatus) {
    unsigned long now = millis();
    if (now - lastRelayCheck >= relayCheckInterval) {
      lastRelayCheck = now;
      isGettingRelayStatus = true;
      Serial.println("Checking Firebase for relayStatus update...");
      Database.get(aClient, "/data/relayStatus", processData, "Check_Relay_Status");
    }
  }
}

void processData(AsyncResult &aResult) {
  if (!aResult.isResult()) return;

  if (aResult.available()) {
    String taskId = aResult.uid();
    String payload = aResult.c_str();
    Serial.printf("Task: %s, Payload: %s\n", taskId.c_str(), payload.c_str());
    Serial.println(taskId);
    Serial.println("'" + payload + "'");
    Serial.println(payload.length());
    
    if (payload == "\nevent: put\ndata: {\"path\":\"/\",\"data\":true}\n") {
        relayStatus = true;
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("Relay re-enabled from Firebase (quick check).");
      }

    if (taskId == "Check_Relay_Status") {
      isGettingRelayStatus = false;
      Serial.println(payload.length());

      if (payload.indexOf("\"data\":true") != -1) {
        relayStatus = true;
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("Relay re-enabled from Firebase (quick check).");
      } else if (payload.indexOf("\"data\":false") != -1) {
        relayStatus = false;
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("Relay disabled from Firebase (quick check).");
      }
      else {
        Serial.println("Invalid payload received for relayStatus.");
      }
    }
  }

  if (aResult.isError()) {
    String taskId = aResult.uid();
    Serial.printf("Error task: %s, msg: %s, code: %d\n", taskId.c_str(), aResult.error().message().c_str(), aResult.error().code());
    if (taskId == "Check_Relay_Status") {
      isGettingRelayStatus = false;  // Ensure flag reset even on failure
    }
  }
}
