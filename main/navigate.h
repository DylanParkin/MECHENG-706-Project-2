#ifndef NAVIGATE_H
#define NAVIGATE_H

#include "base_code.h"
#include "sensors.h"
#include "states.h"

STATE navigating();

enum object_state {
  NO_OBJECT,
  OBSTACLE,
  WALL,
  FIRE
};

float getFlameAngle();
object_state drive(bool);
void RotateOnSpot(float desiredAngle);
void Avoid();
void TrackFlameOnSpot();
#endif  // NAVIGATE_H