#include "navigate.h"

#include "base_code.h"
#include "sensors.h"

bool global_heading_ref_set = false;
float global_heading_ref = 0.0f;
bool fire_close = false;
float ambient[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float obstacle_threshold = 15.0f;

constexpr uint8_t PT_PINS[4] = {A12, A13, A14, A15};
constexpr float SENSOR_ANGLES[4] = {40.0f, 0.0f, 0.0f, -40.0f};
constexpr float DETECTION_THRESHOLD = 0.0f;

STATE navigating() {
  // while (1) {
  //   float asdfda = getFlameAngle();
  //   delay(500);
  // }

  STATE result = drive(true);
  if (result == EXTINGUISH) {
    TrackFlameOnSpot();
  }
  return result;
}

// ============================================================
// getFlameAngle
// ============================================================
float getFlameAngle() {
  constexpr float K = 25.6f;
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

  // SerialCom->print("RAW: ");
  // for (uint8_t i = 0; i < 4; i++) {
  //   SerialCom->print("PT");
  //   SerialCom->print(i);
  //   SerialCom->print(": ");
  //   SerialCom->print(sums[i] / N);
  //   SerialCom->print("  ");
  // }
  // SerialCom->print("| total: ");
  // SerialCom->println(total);

  if (total <= DETECTION_THRESHOLD) {
    filtered = 0.0f;
    SerialCom->println("Angle: NO FLAME");
    return NO_FLAME;
  }

  float asymmetry = (r[0] + r[1] - r[2] - r[3]) / total;
  filtered = alpha * K * asymmetry + (1.0f - alpha) * filtered;

  SerialCom->print("Angle: ");
  SerialCom->println(filtered, 1);
  return filtered;
}

// ============================================================
// object_detected
// ============================================================
object_state object_detected() {
  float left_ir = get_front_left_IR();
  float right_ir = get_front_right_IR();
  float ultrasonic = TriggerUltrasonic();

  float distance_in_front = min(left_ir, min(ultrasonic, right_ir));
  SerialCom->println("L_IR: " + String(left_ir) + " | Ultrasonic: " + String(ultrasonic) + " | R_IR: " + String(right_ir) + " cm");

  if (distance_in_front > obstacle_threshold) {
    return NO_OBJECT;
  }

  uint16_t PT_readings[4];
  for (uint8_t i = 0; i < 4; i++) {
    PT_readings[i] = analogRead(PT_PINS[i]);
  }
  SerialCom->println(String(PT_readings[0]) + " | " + String(PT_readings[1]) + " | " + String(PT_readings[2]) + " | " + String(PT_readings[3]));

  fire_close = (PT_readings[1] + PT_readings[2] > 110);

  uint32_t pt_sum = 0;
  uint16_t pt_max = 0;
  for (uint8_t i = 0; i < 4; i++) {
    pt_sum += PT_readings[i];
    if (PT_readings[i] > pt_max) pt_max = PT_readings[i];
  }

  if (pt_sum < 50) return OBSTACLE;
  if (PT_readings[0] == pt_max || PT_readings[3] == pt_max) return OBSTACLE;
  if (PT_readings[0] + PT_readings[3] < 25) return OBSTACLE;
  return FIRE;
}

// ============================================================
// TrackFlameOnSpot
// ============================================================
void TrackFlameOnSpot() {
  SerialCom->println("TrackFlameOnSpot");

  const float kp = 8.0f;
  const float corrClamp = 350.0f;
  const float minOutput = 67.0f;
  const float deadband = 1.5f;
  const int settleNeeded = 3;

  int settled = 0;

  while (true) {
    float angle = getFlameAngle();

    if (angle == NO_FLAME) {
      stop();
      delay(10);
      continue;
    }

    // SerialCom->println("angle: " + String(angle));

    if (fabsf(angle) < deadband) {
      if (++settled >= settleNeeded) {
        stop();
        SerialCom->println("Flame centred");
        return;
      }
    } else {
      settled = 0;
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
}

// ============================================================
// drive — returns STATE directly
// ============================================================
STATE drive(bool forward) {
  const float kp_fire = 30.0f;
  const float ki_fire = 0.0f;
  const float kd_fire = 0.0f;
  const float integralClamp = 200.0f;
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;

  constexpr int noflame_limit = 50;  // 50 * 10ms = 500ms — tune

  float integralError = 0.0f;
  float prevError = 0.0f;
  int noflame_count = 0;
  int dir = forward ? 1 : -1;

  SettleSensors(10);

  SerialCom->print("Driving ");
  // SerialCom->println(forward ? "forward" : "reverse");

  if (!global_heading_ref_set) {
    global_heading_ref = GetHeading();
    global_heading_ref_set = true;
  }

  unsigned long prev = millis();
  unsigned long drive_start = millis();
  const unsigned long ramp_duration_ms = 1000UL;

  while (true) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

    // --- Priority 1: NOFLAME ---
    float fire_heading = getFlameAngle();
    if (fire_heading == NO_FLAME) {
      if (++noflame_count >= noflame_limit) {
        stop();
        SerialCom->println("FLAME LOST during drive — transitioning to SEARCH");
        return SEARCHING;
      }
      fire_heading = 0.0f;  // hold straight while flame temporarily lost
    } else {
      noflame_count = 0;
    }

    // --- Priority 2: FIRE / OBSTACLE ---
    object_state detected = object_detected();
    if (detected == FIRE) {
      stop();
      SerialCom->println("FIRE DETECTED — stopping");
      return EXTINGUISH;
    } else if (detected == OBSTACLE) {
      stop();
      SerialCom->println("OBSTACLE DETECTED — stopping");
      return AVOID;
    }

    // --- Normal drive with ramp ---
    unsigned long elapsed = now - drive_start;

    float error = fire_heading;
    integralError += error * dt;
    integralError = constrain(integralError,
                              -integralClamp / max(ki_fire, 0.001f),
                              integralClamp / max(ki_fire, 0.001f));
    float deriv = (error - prevError) / dt;
    prevError = error;

    float correction = kp_fire * error + ki_fire * integralError + kd_fire * deriv;
    correction = constrain(correction, -corrClamp, corrClamp);

    if (elapsed < ramp_duration_ms) {
      float ramp_frac = 0.2f + 0.8f * ((float)elapsed / (float)ramp_duration_ms);
      int ramp_speed_val = (int)((float)speed_val * ramp_frac);

      left_front_motor.writeMicroseconds(1500 + dir * ramp_speed_val);
      left_rear_motor.writeMicroseconds(1500 + dir * ramp_speed_val);
      right_rear_motor.writeMicroseconds(1500 - dir * ramp_speed_val);
      right_front_motor.writeMicroseconds(1500 - dir * ramp_speed_val);
    } else {
      left_front_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
      left_rear_motor.writeMicroseconds(1500 + dir * (int)speed_val - (int)correction);
      right_rear_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
      right_front_motor.writeMicroseconds(1500 - dir * (int)speed_val - (int)correction);
    }

    delay(readDelayMs);
  }
}

// ============================================================
// navigating — just propagates drive() result
// ============================================================
