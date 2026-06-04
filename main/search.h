#ifndef SEARCH_H
#define SEARCH_H
#include "base_code.h"
#include "states.h"

STATE searching();

// ---- Hardware ----
extern Servo ultraServo;

// ─── Sweep constants ──────────────────────────────────────────────────────────
extern const int resolution;
extern const int size;
extern float raw_dists[];

// ---- Functions ----
float SpinAndFindFire(float target_spin_angle);
float Spin360AndFindFire();
void RotateOnSpot(float desiredAngle);
// int scanForFire(int &num_fires, int &angle_1, int &angle_2);
float scanFireAngle(int scan_angle);

// ---- Pin Config ----
const int photo_R1_pin = A14;
const int photo_R2_pin = A15;
const int photo_L1_pin = A13;
const int photo_L2_pin = A12;

//----variables----
// bool fire_behind = false;
extern float fire_angle;

#endif  // SEARCH_H