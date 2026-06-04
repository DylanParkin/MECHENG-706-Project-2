#include "avoid.h"

#include "navigate.h"
// THIS IS THE CURRENT AVOID.CPP

STATE avoiding() {
  return Avoid();
}

STATE Avoid() {
  constexpr float low_clearance_threshold = 15.0f;
  constexpr float high_clearance_threshold = 30.0f;
  constexpr float diagonal_approach_threshold = 10.0f;
  constexpr int reverse_speed = 150;  // 100
  constexpr int stuck_reverse_threshold = 300;
  constexpr int stuck_no_clearance_threshold = 300;
  const float kp_fire = 0.0f;
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;

  bool fire_detected = false;

  SerialCom->println("========== AVOID ENTRY ==========");

  while (true) {
    bool strafe_right = false;

    // Priority 1: NOFLAME before anything else
    if (getFlameAngle() == NO_FLAME) {
      SerialCom->println("!! NOFLAME at avoid entry — transitioning to SEARCH");
      stop();
      return SEARCHING;
    }

    // --- Direction Determination ---
    float IR_right = get_right_IR();
    float IR_left = get_left_IR();
    float IR_front_right = get_front_right_IR();
    float IR_front_left = get_front_left_IR();

    SerialCom->println("----- Direction Determination -----");
    // SerialCom->print("  Front L: ");
    // SerialCom->print(IR_front_left);
    // SerialCom->print(" cm  |  Front R: ");
    // SerialCom->print(IR_front_right);
    // SerialCom->println(" cm");
    // SerialCom->print("  Side  L: ");
    // SerialCom->print(IR_left);
    // SerialCom->print(" cm  |  Side  R: ");
    // SerialCom->print(IR_right);
    // SerialCom->println(" cm");

    if (IR_front_left < IR_front_right) {
      strafe_right = true;
      SerialCom->println("  >> STRAFE RIGHT (front sensors: right front sensor clear)");
    } else if (IR_right < 50.0f || IR_left < 50.0f) {
      strafe_right = (IR_right > IR_left);
      SerialCom->print("  >> STRAFE ");
      SerialCom->print(strafe_right ? "RIGHT" : "LEFT");
      SerialCom->println(" (side sensors: strafing towards more open side)");
    } else {
      strafe_right = (getFlameAngle() > 0);
      SerialCom->print("  >> STRAFE ");
      SerialCom->print(strafe_right ? "RIGHT" : "LEFT");
      SerialCom->println(" (head-on: flame heading decides)");
    }

    // --- Strafe Stage ---
    bool front_clearance_was_low = false;
    bool prev_reverse_active = false;
    int consecutive_reverse_count = 0;
    int no_clearance_count = 0;

    SerialCom->println("========== STRAFE STAGE ==========");

    while (true) {
      // Priority 1: FIRE
      if (object_detected() == FIRE) {
        SerialCom->println("  !! FIRE DETECTED during strafe — exiting Avoid");
        stop();
        fire_detected = true;
        break;
      }

      // Priority 2: NOFLAME
      if (getFlameAngle() == NO_FLAME) {
        SerialCom->println("  !! NOFLAME during strafe — transitioning to SEARCH");
        stop();
        return SEARCHING;
      }

      IR_right = get_right_IR();
      IR_left = get_left_IR();

      float side_IR = strafe_right ? IR_right : IR_left;

      if (side_IR < 10.0f) {
        SerialCom->print("  !! SIDE IR TOO CLOSE (");
        SerialCom->print(strafe_right ? "R: " : "L: ");
        SerialCom->print(side_IR);
        SerialCom->println(" cm) — flipping strafe direction");

        strafe_right = !strafe_right;

        float new_clearance_IR = strafe_right ? get_front_left_IR() : get_front_right_IR();
        front_clearance_was_low = (new_clearance_IR < low_clearance_threshold);

        SerialCom->print("  >> Now strafing ");
        SerialCom->println(strafe_right ? "RIGHT" : "LEFT");
        SerialCom->print("  >> New clearance sensor (");
        SerialCom->print(strafe_right ? "Front L" : "Front R");
        SerialCom->print("): ");
        SerialCom->print(new_clearance_IR);
        SerialCom->print(" cm  →  front_clearance_was_low pre-armed: ");
        SerialCom->println(front_clearance_was_low ? "YES" : "NO");
      }

      int dir = strafe_right ? -1 : 1;

      float fire_heading = getFlameAngle();
      float front_clearance_IR = strafe_right ? get_front_left_IR() : get_front_right_IR();
      float approach_IR = strafe_right ? get_front_right_IR() : get_front_left_IR();

      // reverse only when asymmetric — approach side close, clearance side open
      bool reverse_active = (approach_IR < diagonal_approach_threshold);

      int reverse_component = reverse_active ? reverse_speed : 0;

      // Signal 1: consecutive reverse iterations
      consecutive_reverse_count = reverse_active ? consecutive_reverse_count + 1 : 0;

      // Signal 2: strafe iterations without clearance flag ever set
      no_clearance_count = !front_clearance_was_low ? no_clearance_count + 1 : 0;

      // Stuck — NOFLAME already checked above so flame guaranteed in FOV
      if (consecutive_reverse_count >= stuck_reverse_threshold ||
          no_clearance_count >= stuck_no_clearance_threshold) {
        SerialCom->println("  !! STUCK DETECTED");
        SerialCom->print("     Reverse count: ");
        SerialCom->print(consecutive_reverse_count);
        SerialCom->print("  |  No-clearance count: ");
        SerialCom->println(no_clearance_count);
        SerialCom->println("  >> Flame in FOV — TrackFlameOnSpot");
        stop();
        TrackFlameOnSpot();
        return NAVIGATING;
      }

      SerialCom->print("  Clearance (");
      SerialCom->print(strafe_right ? "FL" : "FR");
      SerialCom->print("): ");
      SerialCom->print(front_clearance_IR);
      SerialCom->print(" cm  |  Front Approach (");
      SerialCom->print(strafe_right ? "FR" : "FL");
      SerialCom->print("): ");
      SerialCom->print(approach_IR);
      SerialCom->print(" cm  |  Side (");
      SerialCom->print(strafe_right ? "R" : "L");
      SerialCom->print("): ");
      SerialCom->print(side_IR);
      SerialCom->print(" cm  |  RevCount: ");
      SerialCom->print(consecutive_reverse_count);
      SerialCom->print("  NoClearCount: ");
      SerialCom->println(no_clearance_count);

      if (reverse_active != prev_reverse_active) {
        SerialCom->print("  >> REVERSE BLEND ");
        SerialCom->print(reverse_active ? "ON" : "OFF");
        SerialCom->print(" — Approach IR: ");
        SerialCom->print(approach_IR);
        SerialCom->print(" cm  |  Clearance IR: ");
        SerialCom->println(front_clearance_IR);
        prev_reverse_active = reverse_active;
      }

      if (front_clearance_IR <= low_clearance_threshold && !front_clearance_was_low) {
        front_clearance_was_low = true;
        SerialCom->print("  >> CLEARANCE FLAG SET — obstacle in view (");
        SerialCom->print(front_clearance_IR);
        SerialCom->println(" cm)");
      } else if (front_clearance_was_low && front_clearance_IR >= high_clearance_threshold) {
        SerialCom->print("  >> OBSTACLE CLEARED — clearance IR: ");
        SerialCom->print(front_clearance_IR);
        SerialCom->println(" cm — exiting strafe");
        delay(150);
        stop();
        break;
      }

      float correction = constrain(kp_fire * fire_heading, -corrClamp, corrClamp);

      left_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction - reverse_component);
      left_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction - reverse_component);
      right_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction + reverse_component);
      right_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction + reverse_component);

      delay(readDelayMs);
    }

    if (fire_detected) return NAVIGATING;

    // reorient toward fire before driving forward
    SerialCom->println("========== REORIENTING AFTER STRAFE ==========");
    TrackFlameOnSpot();

    // NOFLAME check after reorient
    if (getFlameAngle() == NO_FLAME) {
      SerialCom->println("!! NOFLAME after reorient — transitioning to SEARCH");
      stop();
      return SEARCHING;
    }

    // --- Forward Drive Stage ---
    bool hit_front_obstacle = false;

    float initial_side_ir = strafe_right ? get_left_IR() : get_right_IR();
    bool side_clearance_was_low = (initial_side_ir > high_clearance_threshold);

    SerialCom->print("  Initial side IR: ");
    SerialCom->print(initial_side_ir);
    SerialCom->print(" cm  —  pre-armed: ");
    SerialCom->println(side_clearance_was_low ? "YES" : "NO");

    SerialCom->println("========== FORWARD DRIVE STAGE ==========");
    SerialCom->print("  Watching side (");
    SerialCom->print(strafe_right ? "L" : "R");
    SerialCom->println(") for obstacle passage");

    while (true) {
      // Priority 1: FIRE
      if (object_detected() == FIRE) {
        SerialCom->println("  !! FIRE DETECTED during forward drive — exiting Avoid");
        stop();
        fire_detected = true;
        break;
      }

      // Priority 2: NOFLAME
      if (getFlameAngle() == NO_FLAME) {
        SerialCom->println("  !! NOFLAME during forward drive — transitioning to SEARCH");
        stop();
        return SEARCHING;
      }

      float front_left = get_front_left_IR();
      float front_right = get_front_right_IR();

      if (front_left < low_clearance_threshold || front_right < low_clearance_threshold) {
        SerialCom->println("  !! FRONT OBSTACLE DETECTED during forward drive");
        SerialCom->print("     FL: ");
        SerialCom->print(front_left);
        SerialCom->print(" cm  |  FR: ");
        SerialCom->print(front_right);
        SerialCom->println(" cm");
        SerialCom->println("  >> Restarting avoid");
        stop();
        hit_front_obstacle = true;
        break;
      }

      float side_clearance_IR = strafe_right ? get_left_IR() : get_right_IR();

      SerialCom->print("  Side (");
      SerialCom->print(strafe_right ? "L" : "R");
      SerialCom->print("): ");
      SerialCom->print(side_clearance_IR);
      SerialCom->print(" cm  |  Front L: ");
      SerialCom->print(front_left);
      SerialCom->print(" cm  |  Front R: ");
      SerialCom->println(front_right);

      if (side_clearance_IR <= low_clearance_threshold && !side_clearance_was_low) {
        side_clearance_was_low = true;
        SerialCom->print("  >> SIDE FLAG SET — obstacle alongside (");
        SerialCom->print(side_clearance_IR);
        SerialCom->println(" cm)");
      } else if (side_clearance_was_low && side_clearance_IR >= high_clearance_threshold) {
        SerialCom->print("  >> OBSTACLE PASSED — side IR: ");
        SerialCom->print(side_clearance_IR);
        SerialCom->println(" cm — exiting forward drive");
        stop();
        break;
      }

      forward();
    }

    if (fire_detected) return NAVIGATING;

    if (!hit_front_obstacle) {
      SerialCom->println("========== AVOID COMPLETE ==========");
      break;
    }

    SerialCom->println("========== AVOID RESTART ==========");
  }

  TrackFlameOnSpot();
  return NAVIGATING;
}