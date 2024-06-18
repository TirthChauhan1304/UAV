#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include <TinyGPS++.h>

// GPS configuration
HardwareSerial gpsSerial(1); // Use UART1 for GPS
TinyGPSPlus gps;

// MPU6050 configuration
MPU6050 mpu;

// HMC5883L configuration
HMC5883L_Simple mag;

// BMP180 configuration
Adafruit_BMP085 bmp;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for GPS

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize HMC5883L
  mag.SetDeclination(23, 35, 'E');
  mag.SetSamplingMode(COMPASS_SINGLE);
  mag.SetScale(COMPASS_SCALE_130);
  mag.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed");
    while (1);
  }
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Get IMU data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to degrees
  float roll = atan2(-ay, az) * RAD_TO_DEG;
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  float yaw = atan2(-gz, gy) * RAD_TO_DEG;

  // Get magnetometer data
  float heading = mag.GetHeadingDegrees();

  // Get altitude from barometer
  float altitude = bmp.readAltitude();

  // Get GPS data
  float latitude = 0.0;
  float longitude = 0.0;
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  // Print sensor data to Serial Monitor
  Serial.print("Roll: "); Serial.print(roll); Serial.print(" ");
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print(" ");
  Serial.print("Yaw: "); Serial.print(yaw); Serial.print(" ");
  Serial.print("Heading: "); Serial.print(heading); Serial.print(" ");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.print(" ");
  Serial.print("Latitude: "); Serial.print(latitude); Serial.print(" ");
  Serial.print("Longitude: "); Serial.println(longitude);

  delay(1000); // Update rate of 1Hz
}
