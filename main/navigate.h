#ifndef NAVIGATE_H
#define NAVIGATE_H

#include "base_code.h"
#include "sensors.h"
#include "states.h"

STATE navigating();

float getFlameAngle();
void drive(bool);
void RotateOnSpot(float desiredAngle);

#endif  // NAVIGATE_H