#include "extinguish.h"
#include "states.h"

bool first_fire = true;

STATE extinguishing(){
    SerialCom->println("Extinguishing... wait 10 seconds...");
    delay(10000);

    if (first_fire) {
        first_fire = false;
        return SEARCHING;
    } else {
        return FINISHED;
    }
}