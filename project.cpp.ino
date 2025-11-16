// -----------------------------------------------------------
// C-ENG-C Senior Design 2024 - Line Following CrawlBot
// Using AFMotor_R4 motor shield + 1 LDR brightness sensor
// Motors: M1 = Left, M2 = ccenter assist, M3 = right 
// ----------------------------------------------------------- 

#include "AFMotor_R4.h"

// ---- Motor Objects ----
AF_DCMotor motorL(2);   // Left motor on M1
AF_DCMotor motorR(3);   // center motor on M3
AF_DCMotor motorC(1);   // Center motor on M3

// ---- LDR Sensor ----
const int LDR_PIN = A0;

// ---- Threshold (tune on real track) ----
int BRIGHTNESS_THRESHOLD = 350;  // change after testing

// ---- Motor speeds (0–255) ----
int SPEED_FORWARD = 180;
int SPEED_TURN    = 120;  // Reduced from 160 for gentler turns
int SPEED_CENTER  = 150;  // Reduced from 180 for less aggressive center

// ---- Center motor pulse control ----

bool centerMotorActive = false;
unsigned long centerMotorStartTime = 0;
const unsigned long CENTER_PULSE_DURATION = 200; // milliseconds

void setup() {
  Serial.begin(9600);
  Serial.println("Line Following Crawler - Starting...");

  // Release all motors at start
  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorC.setSpeed(0);

  motorL.run(RELEASE);
  motorR.run(RELEASE);
  motorC.run(RELEASE);

  delay(1000); // small delay before movement
}

// ---- Helper Functions ----
void driveForward() {
  motorL.setSpeed(SPEED_FORWARD);
  motorR.setSpeed(SPEED_FORWARD);

  motorL.run(FORWARD);
  motorR.run(FORWARD);
  
  // Stop center motor when on track
  stopCenterMotor();
}

void turnRight() {
  motorL.setSpeed(SPEED_TURN);
  motorR.setSpeed(0);

  motorL.run(FORWARD);
  motorR.run(RELEASE);
  
  // Give a short burst with center motor to help turn
  pulseCenterMotor();
}

void turnLeft() {
  motorL.setSpeed(0);
  motorR.setSpeed(SPEED_TURN);

  motorL.run(RELEASE);
  motorR.run(FORWARD);
  
  // Give a short burst with center motor to help turn
  pulseCenterMotor();
}

void pulseCenterMotor() {
  // Only pulse if not already active or if it's been long enough since last pulse
  if (!centerMotorActive || (millis() - centerMotorStartTime > CENTER_PULSE_DURATION * 2)) {
    motorC.setSpeed(SPEED_CENTER);
    motorC.run(FORWARD);
    centerMotorActive = true;
    centerMotorStartTime = millis();
    Serial.println("Center motor PULSE");
  }
}

void stopCenterMotor() {
  motorC.setSpeed(0);
  motorC.run(RELEASE);
  centerMotorActive = false;
}

void checkCenterMotorPulse() {
  // Check if center motor pulse duration has elapsed
  if (centerMotorActive && (millis() - centerMotorStartTime > CENTER_PULSE_DURATION)) {
    stopCenterMotor();
    Serial.println("Center motor STOP");
  }
}

void loop() {
  // Check if center motor pulse needs to be stopped
  checkCenterMotorPulse();

  // Read the brightness from LDR
  int sensorValue = analogRead(LDR_PIN);

  // Print value for tuning threshold
  Serial.print("LDR: ");
  Serial.println(sensorValue);

  // Decide based on sensor
  if (sensorValue < BRIGHTNESS_THRESHOLD) {
    // DARK = on track → go forward (center motor OFF)
    driveForward();
    Serial.println("On track - Forward");
  } else {
    // LIGHT = off track → turn to search (center motor gives short pulse)
    turnRight();   // or turnLeft() if needed
    Serial.println("Off track - Turning");
  }

  delay(5); // short delay to stabilize reading
}
