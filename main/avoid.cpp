#include "avoid.h"

#include "navigate.h"

STATE avoiding() {
  Avoid();

  return NAVIGATING;
}

void Avoid() {
  int dir;
  bool strafe_right = false;
  constexpr float low_clearance_threshold = 11.0f;
  constexpr float high_clearance_threshold = 15.0f;
  // point servo to right and check right ir

  // ultraServo.writeMicroseconds(550);
  // delay(500);

  const float kp_fire = 30.0f;  // was 60
  const float corrClamp = 350.0f;
  const float readDelayMs = 10.0f;

  float ultra_right = TriggerUltrasonic();
  float IR_right = get_right_IR();

  float right_sum = ultra_right + IR_right;

  // ultraServo.writeMicroseconds(2500);
  // delay(500);

  float ultra_left = TriggerUltrasonic();
  float IR_left = get_left_IR();

  float left_sum = ultra_left + IR_left;

  // ultraServo.write(90);
  // delay(300);

  if (right_sum > left_sum) {
    strafe_right = true;
  }

  if (strafe_right) {
    SerialCom->println("strafing right. reading left front");
  } else {
    SerialCom->println("strafing left. reading right front");
  }

  dir = (strafe_right) ? -1 : 1;  // dir +ve strafe left

  bool front_clearance_was_low = false;  // low meaning object in frnt of IR
  bool side_clearance_was_low = false;   // low meaning object in frnt of IR

  // strafe stage
  while (true) {
    float fire_heading = getFlameAngle();

    float front_clearance_IR = (strafe_right) ? get_front_left_IR() : get_front_right_IR();

    SerialCom->print("front clearance IR distance: ");
    SerialCom->print(front_clearance_IR);
    SerialCom->println(" cm");

    // clear object when strafing
    if (front_clearance_IR <= low_clearance_threshold) {
      front_clearance_was_low = true;
    } else if (front_clearance_was_low && front_clearance_IR >= high_clearance_threshold) {
      delay(300);  // strafe a little extra to properly clear obstacle
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
  // while (true) {
  //   float side_clearance_IR = (strafe_right) ? get_left_IR() : get_right_IR();
  //   // clear object when driving forwards

  //   if (side_clearance_IR <= low_clearance_threshold) {
  //     side_clearance_was_low = true;
  //   } else if (side_clearance_was_low && side_clearance_IR >= high_clearance_threshold) {
  //     stop();
  //     return;
  //   }
  //   forward();
  // }
}