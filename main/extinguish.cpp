#include "extinguish.h"
#include "states.h"
#define SWITCH_PIN 15
#define EXTINGUISHED 1

// Analog input pins for PTs {outer_left, inner_left, inner_right, outer_right}
constexpr uint8_t PT_PINS[4] = {A12, A13, A14, A15};

STATE extinguishing(){
    SerialCom->print("EXTINGUISH");
    float start_time = millis();
    
    while (millis() - start_time < 10000 || !fire_extinguished()) {
        digitalWrite(SWITCH_PIN, HIGH);
        
    }

    SerialCom->print("DONE");
    digitalWrite(SWITCH_PIN, LOW);
    return NAVIGATING;
}

bool fire_extinguished() {

    uint16_t PT_readings[4];

    for (uint8_t i = 0; i < 4; i++) {
        PT_readings[i] = analogRead(PT_PINS[i]);
    }

    int pt_min = PT_readings[0];
    int pt_sum = 0;
   
    for (uint8_t i = 0; i < 4; i++) {
      pt_sum += PT_readings[i];
      if (PT_readings[i] < pt_min) pt_min = PT_readings[i];
    }

    if (pt_min < 80 && pt_sum < 2200) {
      return EXTINGUISHED;
    } 
    else {
      return !EXTINGUISHED;
    }
  }
