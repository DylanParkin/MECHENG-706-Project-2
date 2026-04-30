#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Servo.h>

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
float get_left_IR(int ADC_val);
float get_right_IR(int ADC_val);
float get_front_IR(int ADC_val);
float get_back_IR(int ADC_val);

// ─── Ultrasonic ───────────────────────────────────────────────────────────────
float TriggerUltrasonic();

// ─── ISR ──────────────────────────────────────────────────────────────────────
void UltrasonicReturn();

#endif  // SENSORS_H