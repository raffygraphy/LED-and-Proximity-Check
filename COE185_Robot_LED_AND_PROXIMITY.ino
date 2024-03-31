#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "ultrasonic.h"

void setup() {
  Serial.begin(9600);
  initPWM();
  initUltraSonic();
  initLED();
  // sei();  // Enable global interrupts
}


void loop() {
  center_To_Left();
  left_To_Center();
  center_To_Right();
  right_To_Center();
}

// Interrupt Service Routine for Timer 1 Input Capture
// ISR(TIMER1_CAPT_vect) {
//   Serial.print("ISR");
//   measureEchoPulseWidth();
// }
