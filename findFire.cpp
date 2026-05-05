#include "findFire.h"

const int photo_R1_pin = 18;
const int photo_R2_pin = 19;
const int photo_L1_pin = 20;
const int photo_L2_pin = 21;

const int resolution = 5;
const int numSteps = 180 / resolution;

const float threshold = 300.0;     
const float correction_gain = 20.0;

// Store top 2 fires
float bestAngle1 = -1, bestStrength1 = 0;
float bestAngle2 = -1, bestStrength2 = 0;

void scanForFire() {

  // reset
  bestStrength1 = 0;
  bestStrength2 = 0;
  bestAngle1 = -1;
  bestAngle2 = -1;

  for (int i = 0; i < numSteps; i++) {
    int servo_angle = i * resolution;
    ultraServo.write(servo_angle);
    delay(150); //////// maybe decrease??

    float r1 = analogRead(photo_R1_pin); 
    float r2 = analogRead(photo_R2_pin); 
    float l1 = analogRead(photo_L1_pin); 
    float l2 = analogRead(photo_L2_pin); 

    // weighted sensing
    float right = (1.0 * r1) + (0.7 * r2);
    float left  = (1.0 * l1) + (0.7 * l2);
    float total = right + left;

    if (total > threshold) {

      // direction correction
      float error = (right - left) / total;
      float fire_angle = servo_angle + error * correction_gain;

      // clamp
      if (fire_angle < 0) fire_angle = 0;
      if (fire_angle > 180) fire_angle = 180;

      // --- keep top 2 strongest ---
      if (total > bestStrength1) {
        // shift 1 → 2
        bestStrength2 = bestStrength1;
        bestAngle2 = bestAngle1;

        bestStrength1 = total;
        bestAngle1 = fire_angle;

      } else if (total > bestStrength2) {
        bestStrength2 = total;
        bestAngle2 = fire_angle;
      }
    }
  }

  // --- Output ---
  Serial.println("Detected fires:");

  if (bestAngle1 >= 0) {
    Serial.print("Fire 1: ");
    Serial.println(bestAngle1);
  }

  if (bestAngle2 >= 0) {
    Serial.print("Fire 2: ");
    Serial.println(bestAngle2);
  }

  if (bestAngle1 < 0 && bestAngle2 < 0) {
    Serial.println("No fire detected.");
  }
}