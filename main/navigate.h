#ifndef NAVIGATE_H
#define NAVIGATE_H

#include "base_code.h"
#include "sensors.h"
#include "states.h"
#include "base_code.h"
#include "sensors.h"

STATE navigating();

float getFlameAngle();
void drive(bool);
void RotateOnSpot(float desiredAngle);
void Avoid();

#endif  // NAVIGATE_H