#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include <TinyGPS++.h>
#include <RF24.h>

#define throttleServo 23
#define aileronServo  22
#define elevatorServo 21 
#define rudderServo   19

// PWM parameters
const int freq = 50; // Frequency for servo motors
const int resolution = 10; // Higher resolution for more precise control

// Define channels for each servo
const int servoChannel1 = 0;
const int servoChannel2 = 1;
const int servoChannel3 = 2;
const int servoChannel4 = 3;



// NRF24L01 configuration
RF24 radio(27, 26); // CE, CSN pins
const byte address[6] = "00001";

// Command structure
struct Command {
  float rollSetpoint;
  float pitchSetpoint;
  float yawSetpoint;
  float throttle;
};

// Data structure to send back to the ground station
struct SensorData {
  float roll;
  float pitch;
  float yaw;
  float heading;
  float altitude;
  float latitude;
  float longitude;
};

Command command;
SensorData sensorData;

// GPS configuration
HardwareSerial gpsSerial(1); // Use UART1 for GPS
TinyGPSPlus gps;

// MPU6050 configuration
MPU6050 mpu;

// HMC5883L configuration
HMC5883L_Simple mag;

// BMP180 configuration
Adafruit_BMP085 bmp;

// PID constants for stabilization (inner loop)
float Kp_roll = 1.0, Ki_roll = 0.0, Kd_roll = 0.0;
float Kp_pitch = 1.0, Ki_pitch = 0.0, Kd_pitch = 0.0;
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.0;

// Error terms for stabilization
float rollError, pitchError, yawError;
float prevRollError, prevPitchError, prevYawError;
float rollIntegral, pitchIntegral, yawIntegral;
float rollDerivative, pitchDerivative, yawDerivative;

// Timing variables
unsigned long lastTime;
float deltaTime;


void setup() {
  Wire.begin();
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for GPS

   // Set servo pins as output
  pinMode(throttleServo, OUTPUT);
  pinMode(aileronServo , OUTPUT);
  pinMode(elevatorServo , OUTPUT);
  pinMode(rudderServo , OUTPUT);

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization failed");
    while (1);
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

//  // Initialize HMC5883L
//  if (!mag.begin()) {
//    Serial.println("HMC5883L initialization failed");
//    while (1);
//  }

 mag.SetDeclination(23, 35, 'E'); 
 mag.SetSamplingMode(COMPASS_SINGLE);
 mag.SetScale(COMPASS_SCALE_130);
 mag.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed");
    while (1);
  }

  // Configure PWM functionalities for each channel
  ledcSetup(servoChannel1, freq, resolution);
  ledcSetup(servoChannel2, freq, resolution);
  ledcSetup(servoChannel3, freq, resolution);
  ledcSetup(servoChannel4, freq, resolution);

  // Attach the channels to the corresponding GPIOs
  ledcAttachPin(throttleServo, servoChannel1);
  ledcAttachPin(aileronServo, servoChannel2);
  ledcAttachPin(elevatorServo, servoChannel3);
  ledcAttachPin(rudderServo, servoChannel4);
  
  lastTime = millis();
}

void loop() {

  // Get current time
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;

  // Receive commands from ground station
  if (radio.available()) {
    radio.read(&command, sizeof(Command));

    // Update setpoints from received command
    float rollSetpoint = command.rollSetpoint;
    float pitchSetpoint = command.pitchSetpoint;
    float yawSetpoint = command.yawSetpoint;
    ledcWrite(servoChannel1, map(command.throttle, 0, 100, 0, 1024));

  }

  // Read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Get IMU data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to degrees
  sensorData.roll = atan2(-ay, az) * RAD_TO_DEG;
  sensorData.pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  sensorData.yaw = atan2(-gz, gy) * RAD_TO_DEG;

  // Get magnetometer data
  float heading = mag.GetHeadingDegrees();
  sensorData.heading = heading;

  // Get altitude from barometer
  sensorData.altitude = bmp.readAltitude();

  // Get GPS data
  if (gps.location.isValid()) {
    sensorData.latitude = gps.location.lat();
    sensorData.longitude = gps.location.lng();
  } else {
    sensorData.latitude = 0.0;
    sensorData.longitude = 0.0;
  }

  // Calculate stabilization errors
  rollError = command.rollSetpoint - sensorData.roll;
  pitchError = command.pitchSetpoint - sensorData.pitch;
  yawError = command.yawSetpoint - sensorData.yaw;

  // Calculate integrals
  rollIntegral += rollError * deltaTime;
  pitchIntegral += pitchError * deltaTime;
  yawIntegral += yawError * deltaTime;

  // Calculate derivatives
  rollDerivative = (rollError - prevRollError) / deltaTime;
  pitchDerivative = (pitchError - prevPitchError) / deltaTime;
  yawDerivative = (yawError - prevYawError) / deltaTime;

  // PID calculations
  float rollOutput = Kp_roll * rollError + Ki_roll * rollIntegral + Kd_roll * rollDerivative;
  float pitchOutput = Kp_pitch * pitchError + Ki_pitch * pitchIntegral + Kd_pitch * pitchDerivative;
  float yawOutput = Kp_yaw * yawError + Ki_yaw * yawIntegral + Kd_yaw * yawDerivative;

  // Update previous errors
  prevRollError = rollError;
  prevPitchError = pitchError;
  prevYawError = yawError;


 // Servo control - map PID output to servo angles
  int aileronAngle = constrain(map(rollOutput, -45, 45, 0, 1024), 0, 1024);
  int elevatorAngle = constrain(map(pitchOutput, -45, 45, 0, 1024), 0, 1024);
  int rudderAngle = constrain(map(yawOutput, -45, 45, 0, 1024), 0, 1024);

  ledcWrite(servoChannel2, aileronAngle);
  ledcWrite(servoChannel3, elevatorAngle);
  ledcWrite(servoChannel4, rudderAngle);
  
 
  // Send sensor data back to the ground station
  radio.stopListening();
  radio.write(&sensorData, sizeof(SensorData));
  radio.startListening();

  lastTime = currentTime;

  delay(20);  // Update rate of 50Hz
}
