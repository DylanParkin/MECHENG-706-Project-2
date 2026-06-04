#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Servo.h>

#include "base_code.h"
#include "navigate.h"

// sensors.h
#define ECHO_PIN 2
#define TRIG_PIN 5

// ─── Pins ─────────────────────────────────────────────────────────────────────
// 2 - Echo pin on ultrasonic sensor
// 5 - Trigger pin on ultrasonic sensor
// 6 - Signal pin on servo

// A4 - left sensor
// A5 - right sensor
// A6 -  back
// A7 - front

// ─── Servo ────────────────────────────────────────────────────────────────────
extern Servo ultraServo;

// ─── Ultrasonic ISR variables ─────────────────────────────────────────────────
extern volatile unsigned long t_UltraTrigger;
extern volatile unsigned long t_UltraEchoStart;
extern volatile unsigned long t_UltraEchoEnd;
extern volatile int checkStart;
extern volatile int checkEnd;

// ─── Infrared ─────────────────────────────────────────────────────────────────
float get_left_IR();
float get_right_IR();
float get_front_left_IR();
float get_front_right_IR();

// ─── Ultrasonic ───────────────────────────────────────────────────────────────
float TriggerUltrasonic();
float MedianUltrasonic();

// ─── ISR ──────────────────────────────────────────────────────────────────────
void UltrasonicReturn();

// Spam sensors to let filters stabilise and to clear faulty readings. Blocks for fraction of a second
void SettleSensors(int iterations);

#endif  // SENSORS_H