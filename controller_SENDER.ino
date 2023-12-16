#define THROTTLE  35
#define RUDDER  26

int speed = 0;
bool direction = true; // true == forward  false == reverse
int rudderAngle = 90;

void setup() {
  Serial.begin(115200) ;
}

int getSpeed(int reading) {
  if (reading == 0)
    return 255;
  else if (reading < 5)
    return 230;
  else if (reading < 10)
    return 215;
  else if (reading < 14)
    return 200;
  else if (reading < 17)
    return 0;
  else if (reading < 21)
    return 200;
  else if (reading < 26)
    return 215;
  else if (reading < 31)
    return 230;
  else return 255;
}

int getRudder(int reading) {
  if (reading == 0)
    return 130;
  else if (reading < 5)
    return 120;
  else if (reading < 10)
    return 110;
  else if (reading < 14)
    return 100;
  else if (reading < 17)
    return 90;
  else if (reading < 21)
    return 80;
  else if (reading < 26)
    return 70;
  else if (reading < 31)
    return 60;
  else return 50;
}

void loop() {
  // read X and Y analog valueYs
  analogReadResolution(5);
  speed = getSpeed(analogRead(THROTTLE));
  direction = (speed <= 0);
  rudderAngle = getRudder(analogRead(RUDDER));

  Serial.print("speed = ");
  Serial.print(speed);
  Serial.print("        direction = ");
  Serial.print(direction);
  Serial.print("        rudder = ");
  Serial.println(rudderAngle);

}
