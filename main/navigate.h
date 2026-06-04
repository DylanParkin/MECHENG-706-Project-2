#ifndef NAVIGATE_H
#define NAVIGATE_H

#include "base_code.h"
#include "sensors.h"
#include "states.h"

STATE navigating();

enum object_state {
  NO_OBJECT,
  OBSTACLE,
  FIRE
};

constexpr float NO_FLAME = 999.0f;

extern bool global_heading_ref_set;
extern float global_heading_ref;
extern bool fire_close;
extern float ambient[4];
extern float obstacle_threshold;

float getFlameAngle();
STATE drive(bool forward);
object_state object_detected();
void RotateOnSpot(float desiredAngle);
void TrackFlameOnSpot();
#endif  // NAVIGATE_H