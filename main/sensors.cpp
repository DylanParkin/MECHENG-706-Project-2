#include "sensors.h"

#include <math.h>

constexpr float kIrFilterAlpha = 0.5f;

float smooth_ir(float reading, float& filtered, bool& initialized) {
  if (!initialized) {
    filtered = reading;
    initialized = true;
    return filtered;
  }

  filtered += kIrFilterAlpha * (reading - filtered);
  return filtered;
}

volatile unsigned long t_UltraTrigger;
volatile unsigned long t_UltraEchoStart;
volatile unsigned long t_UltraEchoEnd;
volatile int checkStart = 0;
volatile int checkEnd = 0;

float get_left_IR() {  // VALS OK
  int ADC_val = analogRead(A4);
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 16215.0f * pow(ADC_val, -1.194f);
  return smooth_ir(reading, filtered, initialized);
}

float get_right_IR() {  // UPDATED
  int ADC_val = analogRead(A6);
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 12697.0f * pow(ADC_val, -1.167f) + 0.5f;
  return smooth_ir(reading, filtered, initialized);
}

float get_front_left_IR() {
  int ADC_val = analogRead(A5);
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 3616.9f * pow(ADC_val, -1.089f) + 2.0f;
  return smooth_ir(reading, filtered, initialized);
}

float get_front_right_IR() {
  int ADC_val = analogRead(A7);
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 1631.2f * pow(ADC_val, -0.942f) + 1.0f;
  return smooth_ir(reading, filtered, initialized);
}

void UltrasonicReturn() {  // ISR
  if (digitalRead(2) == HIGH) {
    t_UltraEchoStart = micros();
    checkStart = 1;
  } else {
    t_UltraEchoEnd = micros();
    checkEnd = 1;
  }
}

float TriggerUltrasonic() {
  static float last_valid = 999.0f;

  noInterrupts();
  checkStart = 0;
  checkEnd = 0;
  t_UltraEchoStart = 0;
  t_UltraEchoEnd = 0;
  interrupts();

  digitalWrite(5, LOW);
  delayMicroseconds(2);
  digitalWrite(5, HIGH);
  delayMicroseconds(10);
  digitalWrite(5, LOW);

  unsigned long timeoutStart = millis();
  while (!checkEnd && (millis() - timeoutStart) < 10);

  if (checkEnd && checkStart) {
    noInterrupts();
    unsigned long time = t_UltraEchoEnd - t_UltraEchoStart;
    interrupts();

    float dist = time / 58.0f;
    if (dist >= 2.0f && dist <= 400.0f) {
      last_valid = dist;
    }
  }

  delay(50);
  return last_valid;
}

// Commands the sensors many times to let the filtered values stabilise before proceeding
void SettleSensors(int iterations) {
  SerialCom->println("Settling sensors...");
  for (int i = 0; i < iterations; i++) {
    get_left_IR();
    get_right_IR();
    get_front_left_IR();
    get_front_right_IR();
    TriggerUltrasonic();
    SerialCom->println("left IR: " + String(get_left_IR()) + " | front left IR: " + String(get_front_left_IR()) + " | ultrasonic: " + String(TriggerUltrasonic()) + " | front right IR: " + String(get_front_right_IR()) + " | right IR: " + String(get_right_IR()));
  }
}

float MedianUltrasonic() {  // meadian sort over 5 readings
  float window[5];
  for (int i = 0; i < 5; i++) {
    window[i] = TriggerUltrasonic();
  }

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4 - i; j++)
      if (window[j] > window[j + 1]) {
        float tmp = window[j];
        window[j] = window[j + 1];
        window[j + 1] = tmp;
      }

  return window[2];
}