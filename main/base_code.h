#ifndef BASE_CODE_H
#define BASE_CODE_H

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Servo.h>

#include "states.h"

// ─── Compile-time feature flags ───────────────────────────────────────────────
// Uncomment to disable optional hardware:
// #define NO_READ_GYRO
#define NO_HC_SR04
// #define NO_BATTERY_V_OK

// ─── Pin definitions ──────────────────────────────────────────────────────────
extern const byte left_front;
extern const byte left_rear;
extern const byte right_rear;
extern const byte right_front;

extern const int TRIG_PIN;
extern const int ECHO_PIN;

extern const unsigned int MAX_DIST;

// ─── Motor objects ────────────────────────────────────────────────────────────
extern Servo left_front_motor;
extern Servo left_rear_motor;
extern Servo right_rear_motor;
extern Servo right_front_motor;
extern Servo turret_motor;

// ─── IMU ─────────────────────────────────────────────────────────────────────
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern float rad;

// ─── Speed / state variables ─────────────────────────────────────────────────
extern int speed_val;
extern int strafe_speed;
extern int speed_change;
extern int pos;
extern bool start_forward;

// ─── Serial pointer ───────────────────────────────────────────────────────────
extern HardwareSerial* SerialCom;

// ─── State functions ──────────────────────────────────────────────────────────
STATE initialising();
STATE running();
STATE stopped();

// ─── LED helpers ─────────────────────────────────────────────────────────────
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();

// ─── Speed helpers ────────────────────────────────────────────────────────────
void speed_change_smooth();

// ─── Sensor functions ────────────────────────────────────────────────────────
#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK();
#endif

#ifndef NO_HC_SR04
float HC_SR04_range();
#endif

#ifndef NO_READ_GYRO
float GetHeading();
#endif

// ─── Serial command parsing ───────────────────────────────────────────────────
void read_serial_command();

// ─── Motor control ───────────────────────────────────────────────────────────
void enable_motors();
void disable_motors();

void stop();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left();
void strafe_right();

#endif  // BASE_CODE_H