#include "avoid.h"

#include "navigate.h"

STATE avoiding() {
  Avoid();

  return NAVIGATING;
}

// void Avoid() {
//   int dir;
//   bool strafe_right = false;
//   constexpr float low_clearance_threshold = 11.0f;
//   constexpr float high_clearance_threshold = 15.0f;
//   // point servo to right and check right ir

//   // ultraServo.writeMicroseconds(550);
//   // delay(500);

//   const float kp_fire = 0.0f;  // was 30
//   const float corrClamp = 350.0f;
//   const float readDelayMs = 10.0f;

//   float ultra_right = TriggerUltrasonic();
//   float IR_right = get_right_IR();

//   float right_sum = ultra_right + IR_right;

//   // ultraServo.writeMicroseconds(2500);
//   // delay(500);

//   float ultra_left = TriggerUltrasonic();
//   float IR_left = get_left_IR();

//   float left_sum = ultra_left + IR_left;

//   // ultraServo.write(90);
//   // delay(300);

//   if (right_sum > left_sum) {
//     strafe_right = true;
//   }

//   if (strafe_right) {
//     SerialCom->println("strafing right. reading left front");
//   } else {
//     SerialCom->println("strafing left. reading right front");
//   }

//   bool front_clearance_was_low = false;  // low meaning object in frnt of IR
//   bool side_clearance_was_low = false;   // low meaning object in frnt of IR

//   // strafe stage

//   float side_IR = 0.0f;

//   SerialCom->println("AVOIDANCE STRAFE STAGE");
//   while (true) {
//     IR_right = get_right_IR();
//     IR_left = get_left_IR();

//     side_IR = (strafe_right) ? (IR_right) : (IR_left);

//     if (side_IR < 10.0f) {
//       strafe_right = !strafe_right;
//     }

//     dir = (strafe_right) ? -1 : 1;  // dir +ve strafe left

//     float fire_heading = getFlameAngle();

//     float front_clearance_IR = (strafe_right) ? get_front_left_IR() : get_front_right_IR();

//     SerialCom->print("front clearance IR distance: ");
//     SerialCom->print(front_clearance_IR);
//     SerialCom->println(" cm");

//     // clear object when strafing CHECK FOR RISING EDGE
//     if (front_clearance_IR <= low_clearance_threshold) {
//       front_clearance_was_low = true;
//     } else if (front_clearance_was_low && front_clearance_IR >= high_clearance_threshold) {
//       delay(200);  // strafe a little extra to properly clear obstacle
//       stop();
//       break;
//     }

//     float correction = kp_fire * fire_heading;
//     correction = constrain(correction, -corrClamp, corrClamp);

//     left_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction);
//     left_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction);
//     right_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction);
//     right_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction);

//     delay(readDelayMs);
//   }

//   // drive forward stage
//   SerialCom->println("AVOIDANCE DRIVE FORWARD STAGE");
//   while (true) {
//     float side_clearance_IR = (strafe_right) ? get_left_IR() : get_right_IR();
//     // clear object when driving forwards

//     if (side_clearance_IR <= low_clearance_threshold) {
//       side_clearance_was_low = true;
//     } else if (side_clearance_was_low && side_clearance_IR >= high_clearance_threshold) {
//       stop();
//       break;
//     }
//     forward();
//   }

//   TrackFlameOnSpot();
// }

void Avoid() {
  int dir;
  bool strafe_right = false;
  constexpr float low_clearance_threshold = 15.0f;
  constexpr float high_clearance_threshold = 20.0f;
  const float kp_fire = 0.0f;  // was 30
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;

  float IR_right = get_right_IR();
  float IR_left = get_left_IR();
  float IR_front_right = get_front_right_IR();
  float IR_front_left = get_front_left_IR();

  // If the bot is towards one side of the obstacle, continue going around that way
  // Could include a check for the middle range to incorporate going the way the flame is.
  if (IR_front_left < IR_front_right) {
    SerialCom->print("Determined that it shuld strafe right by front sensors FL: ");
    SerialCom->print(IR_front_left);
    SerialCom->print("   FR: ");
    SerialCom->println(IR_front_right);
    strafe_right = true;
  } else if (IR_right < 50.0f || IR_left < 50.0f) {
    //   // Overiding this behaviour is that it should not strafe into something if it is to the left or right of the bot
    //   if (IR_right > IR_left) {
    //     SerialCom->Println("Determined that it should strafe right by side sensors");
    //     strafe_right = true;
    //   }
    SerialCom->println("Should redetermine strafe direction via side sensors. within range to hit something   L: ");
    SerialCom->print(IR_left);
    SerialCom->print("   R: ");
    SerialCom->println(IR_right);
  }

  if (strafe_right) {
    SerialCom->println("strafing right. reading left front");
  } else {
    SerialCom->println("strafing left. reading right front");
  }

  bool front_clearance_was_low = false;  // low meaning object in frnt of IR
  bool side_clearance_was_low = false;   // low meaning object in frnt of IR

  // strafe stage

  float side_IR = 0.0f;

  SerialCom->println("AVOIDANCE STRAFE STAGE");
  while (true) {
    IR_right = get_right_IR();
    IR_left = get_left_IR();

    side_IR = (strafe_right) ? (IR_right) : (IR_left);

    if (side_IR < 10.0f) {
      strafe_right = !strafe_right;
    }

    dir = (strafe_right) ? -1 : 1;  // dir +ve strafe left

    float fire_heading = getFlameAngle();

    float front_clearance_IR = (strafe_right) ? get_front_left_IR() : get_front_right_IR();

    if (front_clearance_IR < low_clearance_threshold) {
      front_clearance_was_low = true;
    }

    SerialCom->print("front clearance IR distance: ");
    SerialCom->print(front_clearance_IR);
    SerialCom->println(" cm");

    // clear object when strafing CHECK FOR RISING EDGE
    if (front_clearance_IR <= low_clearance_threshold) {
      front_clearance_was_low = true;
    } else if (front_clearance_was_low && front_clearance_IR >= high_clearance_threshold) {
      delay(200);  // strafe a little extra to properly clear obstacle
      stop();
      break;
    }

    float correction = kp_fire * fire_heading;
    correction = constrain(correction, -corrClamp, corrClamp);

    left_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction);
    left_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction);
    right_rear_motor.writeMicroseconds(1500 + dir * strafe_speed - correction);
    right_front_motor.writeMicroseconds(1500 - dir * strafe_speed - correction);

    delay(readDelayMs);
  }

  // drive forward stage
  SerialCom->println("AVOIDANCE DRIVE FORWARD STAGE");
  while (true) {
    float side_clearance_IR = (strafe_right) ? get_left_IR() : get_right_IR();
    // clear object when driving forwards
    SerialCom->print("side clearance IR: ");
    SerialCom->println(side_clearance_IR);

    if (side_clearance_IR <= low_clearance_threshold) {
      side_clearance_was_low = true;
    } else if (side_clearance_was_low && side_clearance_IR >= high_clearance_threshold) {
      stop();
      break;
    }
    forward();
  }

  TrackFlameOnSpot();
}