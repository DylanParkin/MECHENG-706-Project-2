#include "navigate.h"

#include "states.h"

STATE navigating() {
  return EXTINGUISH;
}

// drive forward with heading hold
void drive(bool forward) {
  const float kp_gyro = 80.0f;  // was 60
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
  // SerialCom->println("driving...");

  while (true) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

    // --- stop driving condition  ---
    float ir_dist = (forward) ? (get_front_IR(analogRead(A7))) : (get_back_IR(analogRead(A6)));

    if (ir_dist < stopDist) {
      stop();
      // SerialCom->println("Wall reached. Stopping.");
      return;
    }

    // --- compute gyro heading error ---
    float error = heading_error_deg(global_heading_ref, GetHeading());

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

// Analog input pins for the four phototransistors
// Order: outer_left, inner_left, inner_right, outer_right
constexpr uint8_t PT_PINS[4] = {A0, A1, A2, A3};

// Sensor mounting angles in degrees (positive = left, negative = right)
constexpr float SENSOR_ANGLES[4] = {40.0f, 0.0f, 0.0f, -40.0f};

// Calibration values
float ambient[4] = {22.0f, 25.0f, 23.0f, 24.0f};
constexpr float DETECTION_THRESHOLD = 150.0f;
constexpr float BIAS_CORRECTION = 1.0f;  // 1.43f;

// Sentinel value returned when no flame is detected
constexpr float NO_FLAME = 999.0f;

// ============================================================
// Main function: returns angle to flame in degrees
// Positive = flame is to the left, negative = flame is to the right
// Returns NO_FLAME (999.0f) if no flame detected
// ============================================================

float getFlameAngle() {
  // Read all four phototransistors
  uint16_t readings[4];
  for (uint8_t i = 0; i < 4; i++) {
    readings[i] = analogRead(PT_PINS[i]);
  }

  // Subtract ambient, clamp negatives, sum total
  float r[4];
  float total = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    float corrected = (float)readings[i] - ambient[i];
    r[i] = (corrected > 0.0f) ? corrected : 0.0f;
    total += r[i];
  }

  // Detection threshold check
  if (total < DETECTION_THRESHOLD) {
    return NO_FLAME;
  }

  // Weighted centroid
  float numerator = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    numerator += r[i] * SENSOR_ANGLES[i];
  }

  return (numerator / total) * BIAS_CORRECTION;
}