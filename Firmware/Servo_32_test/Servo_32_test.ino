#define SERVO1_PIN 23
#define SERVO2_PIN 22
#define SERVO3_PIN 21
#define SERVO4_PIN 19

// PWM parameters
const int freq = 50; // Frequency for servo motors
const int resolution = 16; // Higher resolution for more precise control
const int minDutyCycle = 3277; // Corresponds to 5% duty cycle
const int maxDutyCycle = 6554; // Corresponds to 10% duty cycle

// Define channels for each servo
const int servoChannel1 = 0;
const int servoChannel2 = 1;
const int servoChannel3 = 2;
const int servoChannel4 = 3;

void setup() {
  // Set servo pins as output
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(SERVO3_PIN, OUTPUT);
  pinMode(SERVO4_PIN, OUTPUT);

  // Configure PWM functionalities for each channel
  ledcSetup(servoChannel1, freq, resolution);
  ledcSetup(servoChannel2, freq, resolution);
  ledcSetup(servoChannel3, freq, resolution);
  ledcSetup(servoChannel4, freq, resolution);

  // Attach the channels to the corresponding GPIOs
  ledcAttachPin(SERVO1_PIN, servoChannel1);
  ledcAttachPin(SERVO2_PIN, servoChannel2);
  ledcAttachPin(SERVO3_PIN, servoChannel3);
  ledcAttachPin(SERVO4_PIN, servoChannel4);
}

void loop() {

  // Sweep the servos from min to max position
  for (int dutyCycle = minDutyCycle; dutyCycle <= maxDutyCycle; dutyCycle++) {
    ledcWrite(servoChannel1, dutyCycle);
    ledcWrite(servoChannel2, dutyCycle);
    ledcWrite(servoChannel3, dutyCycle);
    ledcWrite(servoChannel4, dutyCycle);
    delay(1);
  }

  // Sweep the servos from max to min position
  for (int dutyCycle = maxDutyCycle; dutyCycle >= minDutyCycle; dutyCycle--) {
    ledcWrite(servoChannel1, dutyCycle);
    ledcWrite(servoChannel2, dutyCycle);
    ledcWrite(servoChannel3, dutyCycle);
    ledcWrite(servoChannel4, dutyCycle);
    delay(1);
  }
  
}
