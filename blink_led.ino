// Blink a LED over wifi
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32";
const char* password = "12345678";

// IP Address details
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

#define LED 26

bool blink = LOW;

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_onConnect);
  server.on("/ledblinkon", handle_ledblinkon);
  server.on("/ledblinkoff", handle_ledblinkoff);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  if (blink) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}

void handle_onConnect() {
  blink = LOW;
  Serial.println("LED blinking: OFF");
  server.send(200, "text/HTML", SendHTML(blink));
}

void handle_ledblinkon() {
  blink = HIGH;
  Serial.println("LED blinking: ON");
  server.send(200, "text/HTML", SendHTML(true));
}

void handle_ledblinkoff() {
  blink = LOW;
  Serial.println(("LED blinking: OFF"));
  server.send(200, "text/HTML", SendHTML(false));
}

void handle_NotFound() {
  server.send(200, "text/plain", "Not Found");
}

String SendHTML(bool blink) {
  String ptr = "<!DOCTYPE html><html>\n";
  ptr += "<head><meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, user-scalable = no\">\n";
  ptr += "<title>LED Blinking</title>\n";
  ptr += "<style>html {font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body {margin-top: 50px;} h1 {color: #444444; margin: 50px auto 30px} h3 {color: #444444; margin-bottom: 50px;}\n";
  ptr += ".button {display: block; width: 80px; background-color: #3498db; border: none; color: white; padding 13px 30px; text-decoration: none; font-size: 25px; margin: 0px auto 35px; cursor: pointer; border-radius: 4px;}\n";
  ptr += ".button-on {background-color: #3498db;}\n";
  ptr += ".button-on:active {background-color: #2980b9;}\n";
  ptr += ".button-off {background-color: #34495e;}\n";
  ptr += ".button-off:active {background-color: #2c3e50;}\n";
  ptr += "p {font-size: 14px; color: #888; margin-bottom: 10px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<h1>ESP32 Web Server</h1>\n";
  ptr += "<h3>Using Access Point(AP) Mode</h3>\n";

  if (blink) {
    ptr += "<p>Blink Status: ON</p><a class=\"button button-off\" href=\"/ledblinkoff\">OFF</a>\n";
  } else {
    ptr += "<p>Blink Status: OFF</p><a class=\"button button-on\" href=\"/ledblinkon\">ON</a>\n";
  }

  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}
