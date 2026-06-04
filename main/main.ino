#include <SoftwareSerial.h>

#include "avoid.h"
#include "base_code.h"
#include "extinguish.h"
#include "navigate.h"
#include "search.h"
#include "sensors.h"
#include "states.h"

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
// Bluetooth Serial Port
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

Servo ultraServo;

bool start_forward = true;

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // pins for the IR sensors
  pinMode(A4, INPUT);  // left sensor
  pinMode(A5, INPUT);  // right sensor
  pinMode(A6, INPUT);  // back
  pinMode(A7, INPUT);  // front

  // HOMING
  pinMode(ECHO_PIN, INPUT);   // Connect 'Echo' pin here
  pinMode(TRIG_PIN, OUTPUT);  // Connect 'Trig' pin here
  attachInterrupt(
      digitalPinToInterrupt(2), UltrasonicReturn,
      CHANGE);                      // Interrupt that returns time taken ror ultrasonic echo
  ultraServo.attach(6, 500, 2600);  // Signal pin of Servo
  // ultraServo.write(0);   // reset servo to 0 deg

  // Setup the Serial port and pointer, the pointer allows switching the debug
  // info through the USB port(Serial) or Bluetooth port(Serial1) with ease.

  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("Setup....");
  pinMode(11, OUTPUT);  // Connect 'Echo' pin here

  digitalWrite(11, LOW);

  // delay(1000);  // settling time but no really needed
}

void loop(void)  // main loop
{
  static STATE machine_state = INITIALISING;
  // Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      SerialCom->println("ENTERING INITIALISING STATE");
      machine_state = initialising();
      break;
    case STOPPED:  // Stop if Battery voltage is too low, to protect it
      SerialCom->println("ENTERING STOPPED STATE");
      machine_state = stopped();
      break;
    case SEARCHING:
      SerialCom->println("ENTERING SEARCHING STATE");
      machine_state = searching();
      break;
    case NAVIGATING:
      SerialCom->println("ENTERING NAVIGATING STATE");
      machine_state = navigating();
      break;
    case AVOID:
      SerialCom->println("ENTERING AVOID STATE");
      machine_state = avoiding();
      break;
    case EXTINGUISH:
      SerialCom->println("ENTERING EXTINGUISH STATE");
      machine_state = extinguishing();
      break;
    case FINISHED:
      SerialCom->println("ENTERING FINISHED STATE");
      machine_state = FINISHED;
      stop();
      // idles indefinitely
  };
}
