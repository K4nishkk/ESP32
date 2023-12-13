#define VRX_PIN  26 // X-axis analog input from joystick

int valueX = 0; // to store the X-axis value

// for plotter
// comment out in actual code
int minScale = 0;
int maxScale = 128;

int speed; // duty cycle
bool direction; // true -> forward, false -> reverse

// motor driver connections
int motor1Pin1 = 27; 
int motor1Pin2 = 25; 
int enable1Pin = 32; 

// pwm
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 7; // values range from 0 - 256

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.begin(115200);
}

void loop() {
  // read X analog values
  valueX = (analogRead(VRX_PIN) >> 4);

  if (valueX >= 128) { // forward
    speed = valueX - 128;
    direction = true;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  else { // reverse
    speed = 128 - valueX;
    direction = false;
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  ledcWrite(pwmChannel, speed); 

  // print data to Serial Monitor on Arduino IDE
  Serial.print("speed = ");
  Serial.print(speed);
  Serial.print(", direction = ");
  if (direction) {
    Serial.print("forward");
  }
  else {
    Serial.print("backward");
  }

  // for serial plotter
  Serial.print("  ");
  Serial.print(minScale);
  Serial.print("  ");
  Serial.println(maxScale);

  Serial.print(" ");
}
