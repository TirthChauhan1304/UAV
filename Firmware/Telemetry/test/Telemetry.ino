// Define pins
const int airspeedPin = A0;
const int altitudePin = A1;
const int batteryPin = A2;
const int pitchpin = A3;
const int rollpin = A4;
const int headingpin = A5;



void setup() {
  
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
}

void loop() {
  
  // Simulate test values
  int airspeed = random(0, 100);        // Random value between 0 and 100 for airspeed
  int altitude = random(0, 1000);      // Random value between 0 and 10000 for altitude
  float batteryVoltage = random(900, 1300) / 100.0;  // Random value between 9 and 13 for battery voltage
  int pitch = random(-30, 30);  
  int roll = random(-90, 90);  
  int heading = random(0, 360);  
  
  // Send test values over serial separated by commas
  Serial.print(airspeed);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(batteryVoltage, 2);    // Display two decimal places for voltage
  Serial.print(",");
  Serial.print(pitch);    
  Serial.print(",");
  Serial.print(roll);    
  Serial.print(",");
  Serial.print(heading); 
  Serial.print(",");
  Serial.print("22.3"); 
  Serial.print(",");
  Serial.println("45.7"); 
  
  // Delay for some time
  delay(20);  // Adjust delay as needed
}
