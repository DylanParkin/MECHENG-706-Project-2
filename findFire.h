#ifndef FINDFIRE_H
#define FINDFIRE_H

#include <Arduino.h>
#include <Servo.h>

// ---- Hardware ----
extern Servo ultraServo;

// ---- Core function ----
void scanForFire();

// ---- Configuration (shared if needed) ----
extern const int photo_R1_pin;
extern const int photo_R2_pin;
extern const int photo_L1_pin;
extern const int photo_L2_pin;

#endif