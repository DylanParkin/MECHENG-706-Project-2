#include "navigate.h"

bool global_heading_ref_set = false;
float global_heading_ref = 0.0f;

STATE navigating() {
    // 1. RotateOnSpot until aligned with the heading found in SEARCHING state (this should be in search.cpp)
    //
    // 2. Drive straight until IR/ultrasonic sensor detects an object in front
    //
    // 3. If object is fire (if phototransistor reads above/below threshold), return EXTINGUISH
    //
    // 4. If object is not fire, initiate strafe until no object in front (e.g. if strafing left, until right IR sensor is fully clear of obstacle in front)
    //
    // 5. Record IMU displacement in the "y" direction (perpendicular to the main straight path)
    //
    // 6. Drive forward until appropriate rear IR sensor has fully cleared the obstacle
    //
    // 7. Placeholder for now...

    bool forward = true; // set to always drive forwards
    while (true) {
        bool object_is_fire = drive(forward);
        if (object_is_fire) {
            return EXTINGUISH;
        } else {
            dodge_obstacle(); // involves strafing and driving forwards to clear the obstacle

            adjust_heading(); // robot realigns itself so that it faces the fire
        }
    }
}

float heading_error_deg(float target_heading, float current_heading) {
  float error = target_heading - current_heading;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

// Drives forward until detecting an object. If the object is fire, returns true, if its just an obstacle, returns false
bool drive(bool forward) {
  const float kp_gyro = 80.0f;  // was 60
  const float ki_gyro = 0.0f;
  const float kd_gyro = 0.0f;

  const float integralClamp = 200.0f;
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;
  const float stopDist = 10.5f;

  float integralError = 0.0f;
  float prevError = 0.0f;

  bool object_detected = false;

  if (!global_heading_ref_set) {
    global_heading_ref = GetHeading();
    global_heading_ref_set = true;
  }

  unsigned long prev = millis();
  // SerialCom->println("driving...");

  while (!object_detected) {
    unsigned long now = millis();
    float dt = (now - prev) / 1000.0f;
    prev = now;
    if (dt <= 0.0f) dt = readDelayMs / 1000.0f;

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
    left_front_motor.writeMicroseconds(1500 + (int)speed_val - (int)correction);
    left_rear_motor.writeMicroseconds(1500 + (int)speed_val - (int)correction);
    right_rear_motor.writeMicroseconds(1500 - (int)speed_val - (int)correction);
    right_front_motor.writeMicroseconds(1500 - (int)speed_val - (int)correction);

    // Check if car has reached on obstacle or fire
    if (some ir condition to detect if object in front) {
      if (some condition that reads PTs to see if its fire) {
        return true; // fire
      } else {
        return false; // obstacle
      }
    }
  }
}