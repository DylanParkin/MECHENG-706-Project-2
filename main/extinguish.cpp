#include "extinguish.h"

#include "Arduino.h"
#include "states.h"

#define SWITCH_PIN 11
#define EXTINGUISHED 1

// Analog input pins for PTs {outer_left, inner_left, inner_right, outer_right}
constexpr uint8_t PT_PINS[4] = {A12, A13, A14, A15};
int extinguished_count = 0;

bool first_fire = true;

STATE extinguishing() {
  SerialCom->print("EXTINGUISH");

  if (first_fire) {
    SerialCom->println("fire 1");
  }
  if (!first_fire) {
    SerialCom->println("fire 2");
  }
  float start_time = millis();

  extinguished_count = 0;
  // while (millis() - start_time < 10000 || !fire_extinguished()) {
  while (millis() - start_time < 10000 && extinguished_count < 4) {
    digitalWrite(SWITCH_PIN, HIGH);
    extinguished_count = 0;
    if (fire_extinguished()) {
      for (uint8_t i = 0; i < 4; i++) {
        delay(200);
        if (fire_extinguished()) {
          extinguished_count += 1;
        }
      }
    }
  }
  digitalWrite(SWITCH_PIN, LOW);

  if (first_fire) {
    first_fire = false;
    return SEARCHING;
  } else {
    return FINISHED;
  }
}

bool fire_extinguished() {
  uint16_t PT_readings[4];

  for (uint8_t i = 0; i < 4; i++) {
    PT_readings[i] = analogRead(PT_PINS[i]);
  }

  int pt_sum = 0;

  uint16_t pt_max = 0;
  for (uint8_t i = 0; i < 4; i++) {
    pt_sum += PT_readings[i];
    if (PT_readings[i] > pt_max) pt_max = PT_readings[i];
  }
  if (pt_sum < 50) {
    return EXTINGUISHED;
  } else if (PT_readings[0] == pt_max || PT_readings[3] == pt_max) {
    return EXTINGUISHED;
  } else if (PT_readings[0] + PT_readings[3] < 25) {
    return EXTINGUISHED;
  } else {
    return !EXTINGUISHED;
  }
}
