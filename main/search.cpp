#include "search.h"
#include "states.h"
// #include "findFire.h"

STATE searching(){
  Serial.println("Searching...");

  // initialise
  float fire_angle = -1.0f;

  fire_angle = SpinAndFindFire(360.0);   
  Serial.print("fire_angle: "); 
  Serial.println(fire_angle);    
  delay(100);

  if (fire_angle < 0) {
    fire_behind = true;
    return SEARCHING;
  }

  return NAVIGATING;
}


// const int resolution = 5;
// const int numSteps = 180 / resolution;

const float threshold = 300.0;     
// const float correction_gain = 20.0;


void SpinAndFindFire(float target_spin_angle) {
  const int max_samples = 220;
  const int turn_cmd = 220;  // 140
  const unsigned long timeout_ms = 18000;
  const unsigned long sample_delay_ms = 20;
  // const float min_valid_dist = 3.0f;
  // const float max_valid_dist = 350.0f;

  // float angles_deg[max_samples];
  // float dists_cm[max_samples];
  // int count = 0;

  // ultraServo.write(90);
  // delay(250);

  float prev_heading = GetHeading();
  float accumulated_angle = 0.0f;
  unsigned long start_time = millis();

  // reset
  float best_strength = 0;
  float fire_angle = -1;
 

  while (accumulated_angle < target_spin_angle) {
    if (millis() - start_time > timeout_ms) {
      SerialCom->println("fire scan timeout.");
      break;
    }

    left_front_motor.writeMicroseconds(1500 - turn_cmd);
    left_rear_motor.writeMicroseconds(1500 - turn_cmd);
    right_front_motor.writeMicroseconds(1500 - turn_cmd);
    right_rear_motor.writeMicroseconds(1500 - turn_cmd);

    float curr_heading = GetHeading();
    float delta = curr_heading - prev_heading;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    prev_heading = curr_heading;

    if (delta > 0.0f) accumulated_angle += delta;

    /// -------------- fire detection-----------------------------
    float right = analogRead(photo_R1_pin); // prev r1
    // float r2 = analogRead(photo_R2_pin); 
    float left = analogRead(photo_L1_pin); 
    // float l2 = analogRead(photo_L2_pin); 

    // // weighted sensing
    // float right = (1.0 * r1) + (0.7 * r2);
    // float left  = (1.0 * l1) + (0.7 * l2);
    float total = right + left;

    if (total > threshold) {

      // // direction correction
      // float error = (right - left) / total;
      // float fire_angle = servo_angle + error * correction_gain;

      // clamp
      if (accumulated_angle < 0) accumulated_angle = 0;
      if (accumulated_angle > target_spin_angle) accumulated_angle = target_spin_angle;

      // --- keep strongest (fire location)---
      if (total > best_strength) {

        best_strength = total;
        fire_angle = accumulated_angle;

      }
    }

    // float f = TriggerUltrasonic();
    // if (d >= min_valid_dist && d <= max_valid_dist && count < max_samples) {
    //   float a = accumulated_angle;
    //   while (a >= 360.0f) a -= 360.0f;
    //   angles_deg[count] = a;
    //   dists_cm[count] = d;
    //   count++;
    // }

    delay(sample_delay_ms);
  }

  stop();

  // if (count < 8) {
  //   SerialCom->println("Insufficient scan samples, fallback.");
  //   g_closest_wall_bearing_deg = 0.0f;
  //   g_corner_bearing_deg = 0.0f;
  //   return 0.0f;
  // }

  // // 5-point circular moving average
  // float smooth[max_samples];
  // for (int i = 0; i < count; i++) {
  //   int i0 = (i - 2 + count) % count;
  //   int i1 = (i - 1 + count) % count;
  //   int i3 = (i + 1) % count;
  //   int i4 = (i + 2) % count;
  //   smooth[i] = (dists_cm[i0] + dists_cm[i1] + dists_cm[i] + dists_cm[i3] + dists_cm[i4]) / 5.0f;
  // }
   

  // --- Output ---
  Serial.println("Detected fires:");

  if (fire_angle >= 0) {
    Serial.print("Fire: ");
    Serial.println(fire_angle);
  }

  if (fire_angle < 0 ) {
    Serial.println("No fire detected.");
   }

  // // --- Step 1: absolute maximum = closest fire ---
  // int wall1_i = 0;
  // for (int i = 1; i < count; i++) {
  //   if (smooth[i] < smooth[wall1_i]) wall1_i = i;
  // }
  // float wall1_bearing = angles_deg[wall1_i];
  // if (wall1_bearing > 180.0f) wall1_bearing -= 360.0f;
  // g_closest_wall_bearing_deg = wall1_bearing;
  // if (fire_behind){
  //   fire_angle += 180;
  // }

  return;
}

void RotateOnSpot(float desiredAngle) {
  // Convention: CCW +ve (matches IMU)
  // +90 = rotate 90° CCW (counter-clockwise)
  // -90 = rotate 90° CW (clockwise)
  SerialCom->print("Rotating ");
  SerialCom->print(desiredAngle);
  SerialCom->println(" degrees");

  float integralError = 0.0;
  float prevError = desiredAngle;
  float error;

  const float kp = 5.0;
  const float ki = 0.001;
  const float kd = 0.0;
  float allowableError = 0.5;
  const float readDelayMs = 10.0;
  const float integralClamp = 200.0;
  const float outputClamp = 300.0;
  const float minOutput = 67.0;
  const int settleCount = (abs(desiredAngle) > 90) ? 10 : 5;
  int settledSamples = 0;

  // if (desiredAngle > 160) {
  //   allowableError = 2.0;
  // }

  // snapshot heading at start — all errors relative to this
  float initial_heading = GetHeading();

  unsigned long prev = millis();

  while (true) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

    float currAngle = GetHeading() - initial_heading;
    // wrap to [-180, 180]
    while (currAngle > 180.0f) currAngle -= 360.0f;
    while (currAngle < -180.0f) currAngle += 360.0f;

    error = desiredAngle - currAngle;
    // wrap error to [-180, 180] to take shortest path
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // SerialCom->print("current angle");
    // SerialCom->println(currAngle);
    // SerialCom->print("error");
    // SerialCom->println(error);

    // settle check
    if (abs(error) < allowableError) {
      if (++settledSamples >= settleCount) {
        SerialCom->println("SETTLED. Stopping.");
        stop();
        return;
      }
    } else {
      settledSamples = 0;
    }

    // PID
    integralError += error * dt;
    integralError = constrain(integralError, -integralClamp / ki, integralClamp / ki);
    float derivative = (error - prevError) / dt;
    prevError = error;
    float control_output = kp * error + ki * integralError + kd * derivative;
    control_output = constrain(control_output, -outputClamp, outputClamp);

    if (abs(control_output) > 0 && abs(control_output) < minOutput)
      control_output = copysignf(minOutput, control_output);

    left_front_motor.writeMicroseconds(1500 - (int)control_output);
    left_rear_motor.writeMicroseconds(1500 - (int)control_output);
    right_front_motor.writeMicroseconds(1500 - (int)control_output);
    right_rear_motor.writeMicroseconds(1500 - (int)control_output);

    delay(readDelayMs);
  }
}

// int scanForFire(int &angle_1, int &angle_2) { // variables to 'return' by passing them in as references... also include an angle range to scan?

//   // reset
//   float bestStrength1 = 0;
//   float bestStrength2 = 0;
//   float bestAngle1 = -1;
//   float bestAngle2 = -1;

//   for (int i = 0; i < numSteps; i++) {
//     int angle = i * resolution;
//     ultraServo.write(angle);
//     delay(150); //////// maybe decrease??

//     float r1 = analogRead(photo_R1_pin); 
//     float r2 = analogRead(photo_R2_pin); 
//     float l1 = analogRead(photo_L1_pin); 
//     float l2 = analogRead(photo_L2_pin); 

//     // weighted sensing
//     float right = (1.0 * r1) + (0.7 * r2);
//     float left  = (1.0 * l1) + (0.7 * l2);
//     float total = right + left;

//     if (total > threshold) {

//       // direction correction
//       float error = (right - left) / total;
//       float fire_angle = servo_angle + error * correction_gain;

//       // clamp
//       if (fire_angle < 0) fire_angle = 0;
//       if (fire_angle > 180) fire_angle = 180;

//       // --- keep top 2 strongest ---
//       if (total > bestStrength1) {
//         // shift 1 → 2
//         bestStrength2 = bestStrength1;
//         bestAngle2 = bestAngle1;

//         bestStrength1 = total;
//         bestAngle1 = fire_angle;

//       } else if (total > bestStrength2) {
//         bestStrength2 = total;
//         bestAngle2 = fire_angle;
//       }
//     }
//   }

//   // --- Output ---
//   Serial.println("Detected fires:");

//   if (bestAngle1 >= 0) {
//     Serial.print("Fire 1: ");
//     Serial.println(bestAngle1);
//   }

//   if (bestAngle2 >= 0) {
//     Serial.print("Fire 2: ");
//     Serial.println(bestAngle2);
//   }

//   if (bestAngle1 < 0 && bestAngle2 < 0) {
//     Serial.println("No fire detected.");
//   }
// }