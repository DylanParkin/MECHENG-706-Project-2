#include "sensors.h"

#include <math.h>

constexpr float kIrFilterAlpha = 0.25f;

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

float get_left_IR(int ADC_val) {  // VALS OK
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 14307.0f * pow(ADC_val, -1.168f);
  return smooth_ir(reading, filtered, initialized);
}

float get_right_IR(int ADC_val) {  // UPDATED
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 12802.0f * pow(ADC_val, -1.151f);
  return smooth_ir(reading, filtered, initialized);
}

float get_front_IR(int ADC_val) {
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 2790.8 * pow(ADC_val, -1.03);
  return smooth_ir(reading, filtered, initialized);
}

float get_back_IR(int ADC_val) {
  static float filtered = 0.0f;
  static bool initialized = false;
  ADC_val = max(1, ADC_val);
  float reading = 1880.5 * pow(ADC_val, -0.957);
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
  noInterrupts();
  checkStart = 0;
  checkEnd = 0;
  t_UltraEchoStart = 0;
  t_UltraEchoEnd = 0;
  interrupts();

  int echo_time_out_ms = 10;  // specified 30
  int cycle_delay_ms = 15;    // specified 60

  digitalWrite(5, LOW);
  delayMicroseconds(2);
  digitalWrite(5, HIGH);
  delayMicroseconds(10);
  digitalWrite(5, LOW);

  unsigned long timeoutStart = millis();
  while (!checkEnd && (millis() - timeoutStart) < echo_time_out_ms);

  float dist = -1.0f;
  if (checkEnd) {
    noInterrupts();
    unsigned long time = t_UltraEchoEnd - t_UltraEchoStart;
    interrupts();
    dist = time / 58.0f;
  }

  delay(cycle_delay_ms);
  return dist;
}
