#include "search.h"
// #include "findFire.h"

STATE searching() {
  SerialCom->println("Searching...");

  // initialise
  float fire_angle = -1.0f;

  fire_angle = Spin360AndFindFire();
  SerialCom->print("fire_angle: ");
  SerialCom->println(fire_angle);
  RotateOnSpot(fire_angle);

  return NAVIGATING;
}

const float threshold = 20.0;
// const float correction_gain = 20.0;

float Spin360AndFindFire() {
  const int max_samples = 220;
  const int turn_cmd = 220;  // 140
  const unsigned long timeout_ms = 18000;
  const unsigned long sample_delay_ms = 20;
  // const float min_valid_dist = 3.0f;
  // const float max_valid_dist = 350.0f;

  float angles_deg[max_samples];
  float fire_total[max_samples];
  float fire_angle[10];
  int count = 0;

  float prev_heading = GetHeading();
  float accumulated_angle = 0.0f;
  unsigned long start_time = millis();

  // reset
  float best_strength = 0;
  fire_angle[0] = -1;
  int max_fire_i = 0;

  while (accumulated_angle < 360) {
    if (millis() - start_time > timeout_ms) {
      SerialCom->println("fire scan timeout.");
      break;
    }

    left_front_motor.writeMicroseconds(1500 - turn_cmd);
    left_rear_motor.writeMicroseconds(1500 - turn_cmd);
    right_front_motor.writeMicroseconds(1500 - turn_cmd);
    right_rear_motor.writeMicroseconds(1500 - turn_cmd);

    float curr_heading = GetHeading();
    SerialCom->print("current heading: ");
    SerialCom->println(curr_heading);
    float delta = curr_heading - prev_heading;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    prev_heading = curr_heading;

    if (delta > 0.0f) accumulated_angle += delta;

    /// -------------- fire detection-----------------------------
    float right = analogRead(photo_R1_pin);  // prev r1
    // float r2 = analogRead(photo_R2_pin);
    float left = analogRead(photo_L1_pin);
    // float l2 = analogRead(photo_L2_pin);

    // // weighted sensing
    // float right = (1.0 * r1) + (0.7 * r2);
    // float left  = (1.0 * l1) + (0.7 * l2);
    float total = right + left;

    // if (total > threshold) {

    // // direction correction
    // float error = (right - left) / total;
    // float fire_angle = servo_angle + error * correction_gain;

    // clamp
    if (accumulated_angle < 0) accumulated_angle = 0;
    if (accumulated_angle > 360) accumulated_angle = 360;

    // --- keep strongest (fire location)---
    if (total > best_strength) {
      best_strength = total;
      fire_angle[max_fire_i] = accumulated_angle;
      max_fire_i = 0;
    }

    if (total == best_strength) {
      max_fire_i++;
      fire_angle[max_fire_i] = accumulated_angle;
    }
    // }

    /// --------------for storing the values in an array ---------------

    if (count < max_samples) {  // consider also implementing a maximum and minimum reading for total (fire strength)
      float a = accumulated_angle;
      while (a >= 360.0f) a -= 360.0f;
      angles_deg[count] = a;
      fire_total[count] = total;
      count++;
    }

    delay(sample_delay_ms);
  }

  stop();

  // if (count < 8) {
  //   SerialCom->println("Insufficient scan samples, fallback.");
  //   g_closest_wall_bearing_deg = 0.0f;
  //   g_corner_bearing_deg = 0.0f;
  //   return 0.0f;
  // }

  // 5-point circular moving average
  float smooth[count];
  for (int i = 0; i < count; i++) {
    int i0 = (i - 2 + count) % count;
    int i1 = (i - 1 + count) % count;
    int i3 = (i + 1) % count;
    int i4 = (i + 2) % count;
    smooth[i] = (fire_total[i0] + fire_total[i1] + fire_total[i] + fire_total[i3] + fire_total[i4]) / 5.0f;
  }

  // gradient?? - get minimum gradient when not = 0?
  int fire_threshold = 10;
  int min_gradient = 100;     // arbitrary large number
  float min_grad_angle = -1;  // angle of minimum gradient
  for (int i = 0; i < count; i++) {
    if (smooth[i] > fire_threshold) {
      int gradient = smooth[i] - smooth[(i - 1 + count) % count];
      if (abs(gradient) < min_gradient) {
        min_gradient = abs(gradient);
        min_grad_angle = angles_deg[i];
      }
    }
  }

  // --- Output ---
  SerialCom->println("Detected fires:");

  if (fire_angle >= 0) {
    SerialCom->print("Fire: ");
    SerialCom->println(fire_angle[0]);
  }

  if (fire_angle < 0) {
    SerialCom->println("No fire detected.");
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

  // ---------- CHOOSE ONE OR THE OTHER TO OUTPUT ------------------

  // float return_fire_angle = fire_angle[0];
  // ------------ or use median? ---------------
  float return_fire_angle = fire_angle[(max_fire_i + 1) / 2];  // rounds down to nearest int as they're integers

  return return_fire_angle;
}

float SpinAndFindFire(float target_spin_angle) {
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
    float right = analogRead(photo_R1_pin);  // prev r1
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
  SerialCom->println("Detected fires:");

  if (fire_angle >= 0) {
    SerialCom->print("Fire: ");
    SerialCom->println(fire_angle);
  }

  if (fire_angle < 0) {
    SerialCom->println("No fire detected.");
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

  return fire_angle;
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

float scanFireAngle(int scan_angle) {
  // reset
  float bestStrength = 0;
  float bestAngle = -1;

  const int resolution = 5;
  const int numSteps = scan_angle / resolution;
  float correction_gain = 0.02;  // TUNE for angle resolution maybe

  for (int i = 0; i < numSteps; i++) {
    int servo_angle = i * resolution;
    ultraServo.write(servo_angle);
    delay(150);  //////// maybe decrease??

    float r1 = analogRead(photo_R1_pin);
    float r2 = analogRead(photo_R2_pin);
    float l1 = analogRead(photo_L1_pin);
    float l2 = analogRead(photo_L2_pin);

    // weighted sensing
    float right = (1.0 * r1) + (0.7 * r2);
    float left = (1.0 * l1) + (0.7 * l2);
    float total = right + left;

    if (total > threshold) {
      // direction correction
      float error = (right - left) / total;
      float fire_angle = servo_angle + error * correction_gain;

      // clamp
      if (fire_angle < 0) fire_angle = 0;
      if (fire_angle > 180) fire_angle = 180;

      // --- keep strongest ---
      if (total > bestStrength) {
        bestStrength = total;
        bestAngle = fire_angle;
      }
    }
  }

  // --- Output ---
  SerialCom->println("Detected fire:");

  if (bestAngle >= 0) {
    SerialCom->print("Fire: ");
    SerialCom->println(bestAngle);
  }

  if (bestAngle < 0) {
    SerialCom->println("No fire detected.");
  }
}