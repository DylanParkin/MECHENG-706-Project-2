#include "navigate.h"

bool global_heading_ref_set = false;
float global_heading_ref = 0.0f;

// Analog input pins for PTs {outer_left, inner_left, inner_right, outer_right}
constexpr uint8_t PT_PINS[4] = {A12, A13, A14, A15};

bool fire_close = false;

STATE navigating() {
  while (1) {
    TrackFlameOnSpot();
    delay(5000);
  }
  while (1) {
    object_state object = drive(true);
    if (object == FIRE) {
      return EXTINGUISH;
    } else if (object == OBSTACLE) {
      return AVOID;
    }
  }
}

// ============================================================
// Configuration
// ============================================================

// sensor mounting angles in degrees (positive = left, negative = right)
constexpr float SENSOR_ANGLES[4] = {40.0f, 0.0f, 0.0f, -40.0f};

// calibration vals
float ambient[4] = {0.0f, 0.0f, 0.0f, 0.0f};
constexpr float DETECTION_THRESHOLD = 0.0f;
constexpr float NO_FLAME = 999.0f;

// ============================================================
// Main function: returns angle to flame in degrees
// Positive = flame is to the left, negative = flame is to the right
// Returns NO_FLAME (999.0f) if no flame detected
// ============================================================

constexpr float K = 25.6f;  // calibration constant

// float getFlameAngle() {
//   uint16_t readings[4];
//   for (uint8_t i = 0; i < 4; i++) {
//     readings[i] = analogRead(PT_PINS[i]);
//   }

//   // Subtract ambient
//   float r[4];
//   for (uint8_t i = 0; i < 4; i++) {
//     float corrected = (float)readings[i] - ambient[i];
//     r[i] = (corrected > 0.0f) ? corrected : 0.0f;
//   }

//   // Group by side: indices 0,1 are left; indices 2,3 are right
//   float left_signal = r[0] + r[1];
//   float right_signal = r[2] + r[3];
//   float total = left_signal + right_signal;

//   // Detection threshold
//   if (total <= DETECTION_THRESHOLD) {
//     return NO_FLAME;
//   }

//   // Normalized asymmetry → bearing
//   float asymmetry = (left_signal - right_signal) / total;

//   // SerialCom->println(K * asymmetry);
//   return K * asymmetry;
// }

float getFlameAngle() {
  static float filtered = 0.0f;
  const float alpha = 0.5f;
  const int N = 3;

  uint32_t sums[4] = {0, 0, 0, 0};
  for (int s = 0; s < N; s++) {
    for (uint8_t i = 0; i < 4; i++) {
      sums[i] += analogRead(PT_PINS[i]);
    }
  }

  float r[4], total = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    float c = (float)sums[i] / N - ambient[i];
    r[i] = (c > 0.0f) ? c : 0.0f;
    total += r[i];
  }

  if (total <= DETECTION_THRESHOLD) {
    filtered = 0.0f;
    return NO_FLAME;
  }

  float asymmetry = (r[0] + r[1] - r[2] - r[3]) / total;
  filtered = alpha * K * asymmetry + (1.0f - alpha) * filtered;
  return filtered;
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

    fire_close = (PT_readings[1] + PT_readings[2] > 110) ? true : false;

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
  const float kp_fire = 30.0f;
  const float ki_fire = 0.0f;
  const float kd_fire = 0.0f;

  const float integralClamp = 200.0f;
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;

  float integralError = 0.0f;
  float prevError = 0.0f;

  int dir = forward ? 1 : -1;

  // Let sensors settle before starting movement to avoid reacting to spurious/old readings
  SettleSensors(30);

  SerialCom->print("Driving ");
  SerialCom->println(forward ? "forward" : "reverse");

  if (!global_heading_ref_set) {
    global_heading_ref = GetHeading();
    global_heading_ref_set = true;
  }

  unsigned long prev = millis();
  unsigned long drive_start = millis();
  const unsigned long ramp_duration_ms = 1000UL;  // ramp lasts 1 second

  while (true) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

    float fire_heading = getFlameAngle();

    // CHECK FOR OBSTACLE — active even during ramp
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

    unsigned long elapsed = now - drive_start;

    if (elapsed < ramp_duration_ms) {
      // --- RAMP PHASE: straight, no correction, speed 0.2 -> 1.0 over 1 second ---
      float ramp_frac = 0.2f + 0.8f * ((float)elapsed / (float)ramp_duration_ms);
      int ramp_speed_val = (float)speed_val * ramp_frac;

      left_front_motor.writeMicroseconds(1500 + dir * ramp_speed_val);
      left_rear_motor.writeMicroseconds(1500 + dir * ramp_speed_val);
      right_rear_motor.writeMicroseconds(1500 - dir * ramp_speed_val);
      right_front_motor.writeMicroseconds(1500 - dir * ramp_speed_val);
    } else {
      // --- NORMAL PHASE: full speed with fire-tracking correction ---
      float error = fire_heading;

      integralError += error * dt;
      integralError = constrain(integralError,
                                -integralClamp / max(ki_fire, 0.001f),
                                integralClamp / max(ki_fire, 0.001f));
      float deriv = (error - prevError) / dt;
      prevError = error;

      float correction = kp_fire * error + ki_fire * integralError + kd_fire * deriv;
      correction = constrain(correction, -corrClamp, corrClamp);

      left_front_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
      left_rear_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
      right_rear_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
      right_front_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
    }
  }
}

// void TrackFlameOnSpot() {
//   SerialCom->println("AVOIDANCE TrackFLameOnSpot");
//   const float kp_fire = 10.0f;
//   const float ki_fire = 0.0f;
//   const float kd_fire = 0.0f;

//   const float integralClamp = 200.0f;
//   const float corrClamp = 350.0f;
//   const float minOutput = 67.0;
//   const float readDelayMs = 10.0f;

//   float integralError = 0.0f;
//   float prevError = 0.0f;

//   unsigned long prev = millis();

//   while (1) {
//     unsigned long now = millis();
//     float dt = (now - prev) / 1000.0f;
//     prev = now;
//     if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

//     float fire_heading = getFlameAngle();
//     // --- compute fire heading error ---
//     float error = fire_heading;

//     uint16_t PT_readings[4];
//     SerialCom->print("pts: ");
//     for (uint8_t i = 0; i < 4; i++) {
//       PT_readings[i] = analogRead(PT_PINS[i]);
//     }
//     SerialCom->println(String(PT_readings[0]) + " | " + String(PT_readings[1]) + " | " + String(PT_readings[2]) + " | " + String(PT_readings[3]));

//     if (abs(error) < 5.0f) {
//       SerialCom->println("AVOIDANCE rotation end");
//       stop();
//       return;
//     }

//     integralError += error * dt;
//     integralError = constrain(integralError,
//                               -integralClamp / max(ki_fire, 0.001f),
//                               integralClamp / max(ki_fire, 0.001f));
//     float deriv = (error - prevError) / dt;
//     prevError = error;

//     float correction = kp_fire * error + ki_fire * integralError + kd_fire * deriv;

//     correction = constrain(correction, -corrClamp, corrClamp);

//     if (abs(correction) > 0 && abs(correction) < minOutput)
//       correction = copysignf(minOutput, correction);

//     // +correction (turn too much cw) = command CCW (- - - -)
//     left_front_motor.writeMicroseconds(1500 - (int)correction);
//     left_rear_motor.writeMicroseconds(1500 - (int)correction);
//     right_rear_motor.writeMicroseconds(1500 - (int)correction);
//     right_front_motor.writeMicroseconds(1500 - (int)correction);

//     delay(readDelayMs);
//   }
// }

void TrackFlameOnSpot() {
  // SerialCom->println("TrackFlameOnSpot");

  const float kp = 8.0f;
  const float corrClamp = 350.0f;
  const float minOutput = 67.0f;
  const float deadband = 1.5f;  // deg from getFlameAngle()
  const int settleNeeded = 3;   // consecutive readings inside deadband to confirm centred
  const unsigned long timeoutMs = 3000;

  int settled = 0;
  unsigned long start = millis();

  while (1) {  // millis() - start < timeoutMs
    float angle = getFlameAngle();

    // Don't use NO_FLAME as an error signal
    if (angle == NO_FLAME) {
      stop();
      delay(10);
      continue;
    }

    SerialCom->println("angle: " + String(angle));

    if (fabsf(angle) < deadband) {
      if (++settled >= settleNeeded) {
        stop();
        SerialCom->println("Flame centred");
        return;
      }
    } else {
      settled = 0;  // reset if we drift back out
    }

    float correction = constrain(kp * angle, -corrClamp, corrClamp);

    if (fabsf(correction) > 0.0f && fabsf(correction) < minOutput)
      correction = copysignf(minOutput, correction);

    left_front_motor.writeMicroseconds(1500 - (int)correction);
    left_rear_motor.writeMicroseconds(1500 - (int)correction);
    right_rear_motor.writeMicroseconds(1500 - (int)correction);
    right_front_motor.writeMicroseconds(1500 - (int)correction);

    delay(10);
  }

  stop();
  SerialCom->println("TrackFlameOnSpot timeout");
}
