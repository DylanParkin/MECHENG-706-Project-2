#include "navigate.h"

bool global_heading_ref_set = false;
float global_heading_ref = 0.0f;

// Analog input pins for PTs {outer_left, inner_left, inner_right, outer_right}
constexpr uint8_t PT_PINS[4] = {A12, A13, A14, A15};

STATE navigating() {
  while (1) {
    object_state object = drive(true);
    if (object == FIRE) {
      return FINISHED;
    } else if (object == OBSTACLE) {
      return AVOID;
    }
  }
}

object_state object_detected() {
    float left_ir = get_front_left_IR();
    float ultrasonic = TriggerUltrasonic();
    float right_ir = get_front_right_IR();
    float distance_in_front = min(left_ir, min(ultrasonic, right_ir));
    SerialCom->println("L_IR: " + String(left_ir) + " | Ultrasonic: " + String(ultrasonic) + " | R_IR: " + String(right_ir) + " cm");
    float obstacle_threshold = 15.0f;
    uint16_t PT_readings[4];

    if (distance_in_front > obstacle_threshold) {
        return NO_OBJECT;
    } else {
        for (uint8_t i = 0; i < 4; i++) {
            PT_readings[i] = analogRead(PT_PINS[i]);
        }
        SerialCom->println(String(PT_readings[0]) + " | " + String(PT_readings[1]) + " | " + String(PT_readings[2]) + " | " + String(PT_readings[3]));

        uint32_t pt_sum = 0;
        uint16_t pt_max = 0;
        for (uint8_t i = 0; i < 4; i++) {
            pt_sum += PT_readings[i];
            if (PT_readings[i] > pt_max) pt_max = PT_readings[i];
        }

        if (pt_sum < 50) {
            return OBSTACLE;
        } else if (PT_readings[0] == pt_max || PT_readings[3] == pt_max) {
            return OBSTACLE;
        } else if (PT_readings[0] + PT_readings[3] < 25) {
            return OBSTACLE;
        } else {
            return FIRE;
        }
    }
}

// drive forward with fire tracking
object_state drive(bool forward) {
  const float kp_gyro = 30.0f;  // was 60
  const float ki_gyro = 0.0f;
  const float kd_gyro = 0.0f;

  const float integralClamp = 200.0f;
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;
  const float stopDist = 10.5f;

  float integralError = 0.0f;
  float prevError = 0.0f;

  int dir = forward ? 1 : -1;

  SerialCom->print("Driving ");
  SerialCom->println(forward ? "forward" : "reverse");

  // Use one heading reference across the full tilling pass to avoid
  // segment-to-segment yaw bias accumulation.
  if (!global_heading_ref_set) {
    global_heading_ref = GetHeading();
    global_heading_ref_set = true;
  }

  unsigned long prev = millis();

  while (true) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

    float current_heading = GetHeading();
    float fire_heading = getFlameAngle();

    // CHECK FOR OBSTACLE
    object_state detected = object_detected();
    if (detected == FIRE) {
      SerialCom->println("FIRE DETECTED! STOPPING.");
      stop();
      return FIRE;
    } else if (detected == OBSTACLE) {
      SerialCom->println("OBSTACLE DETECTED! STOPPING.");
      stop();
      return OBSTACLE;
    }

    // --- compute gyro heading error ---
    float error = fire_heading;  // heading_error_deg(fire_heading, current_heading);

    // SerialCom->print("heading error: ");
    // SerialCom->println(error);
    integralError += error * dt;
    integralError = constrain(integralError,
                              -integralClamp / max(ki_gyro, 0.001f),
                              integralClamp / max(ki_gyro, 0.001f));
    float deriv = (error - prevError) / dt;
    prevError = error;

    // Gyro-only heading hold for straight-line travel.
    float correction = kp_gyro * error + ki_gyro * integralError + kd_gyro * deriv;

    correction = constrain(correction, -corrClamp, corrClamp);

    // +correction (turn too much cw) = command CCW (- - - -)
    left_front_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
    left_rear_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
    right_rear_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
    right_front_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
  }
}

// ============================================================
// Configuration
// ============================================================

// sensor mounting angles in degrees (positive = left, negative = right)
constexpr float SENSOR_ANGLES[4] = {40.0f, 0.0f, 0.0f, -40.0f};

// calibration vals
float ambient[4] = {30.0f, 30.0f, 30.0f, 30.0f};
constexpr float DETECTION_THRESHOLD = 150.0f;
constexpr float NO_FLAME = 999.0f;

// ============================================================
// Main function: returns angle to flame in degrees
// Positive = flame is to the left, negative = flame is to the right
// Returns NO_FLAME (999.0f) if no flame detected
// ============================================================

constexpr float K = 25.6f;  // calibration constant

float getFlameAngle() {
  uint16_t readings[4];
  for (uint8_t i = 0; i < 4; i++) {
    readings[i] = analogRead(PT_PINS[i]);
  }

  // Subtract ambient
  float r[4];
  for (uint8_t i = 0; i < 4; i++) {
    float corrected = (float)readings[i] - ambient[i];
    r[i] = (corrected > 0.0f) ? corrected : 0.0f;
  }

  // SerialCom->print("| LEFT: ");
  // SerialCom->print(r[0]);
  // SerialCom->print(" | LEFT MID: ");
  // SerialCom->print(r[1]);
  // SerialCom->print(" | RIGHT MID: ");
  // SerialCom->print(r[2]);
  // SerialCom->print(" | RIGHT: ");
  // SerialCom->println(r[3]);

  // Group by side: indices 0,1 are left; indices 2,3 are right
  float left_signal = r[0] + r[1];
  float right_signal = r[2] + r[3];
  float total = left_signal + right_signal;

  // Detection threshold
  if (total < DETECTION_THRESHOLD) {
    return NO_FLAME;
  }

  // Normalized asymmetry → bearing
  float asymmetry = (left_signal - right_signal) / total;

  SerialCom->println(K * asymmetry);
  return K * asymmetry;
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

  while (!object_detected) {
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
