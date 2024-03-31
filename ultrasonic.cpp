// #include "ultrasonic.h"

// void initPWM() {
//   // MODE TIMER
//   TCCR1B &= ~(1 << WGM13);
//   TCCR1B |= (1 << WGM12);

//   // PWM MODE 15BIT OF THE TIMER
//   TCCR1A |= (1 << WGM13) | (1 << WGM12) | (1 << WGM11) | (1 << WGM10);
//   //   // MODE TIMER
//   // TCCR1B &= ~(1 << WGM13);
//   // TCCR1B |= (1 << WGM12);

//   // // PWM MODE 10BIT OF THE TIMER
//   // TCCR1A |= (1 << WGM11) | (1 << WGM10);

//   // PWM MODE 10
//   TCCR1A |= (1 << COM1A1);
//   TCCR1A &= ~(1 << COM1A0);  //NON INVERTING MODE

//   // Set prescaler to 64
//   TCCR1B |= (1 << CS11) | (1 << CS10);  // 12,11,10 - 100

//   /*  
//   System clock is 16Mhz sdand prescalar is "n" for generating clock of 1s
//   Speed of Timer1 = 16MHz/8 = 2MHz, Pulse Time= 1/2MHz KHz = 0.5μs
//   Speed of Timer1 = 16MHz/256 = 62.5 KHz, Pulse Time= 1/62.5 KHz = 16μs
//   Speed of Timer1 = 16MHz/64 = 250kHz(clock cycle/sec), Pulse Time= 1/250kHz KHz = 4μs
//   Speed of Timer1 = 16MHz/256 = 62.5 KHz, Pulse Time= 1/62.5 KHz = 16μs
// */

//   // Set OC1A (Pin 9) as output == DIG9/PIN 9 in arduino and based on board
//   DDRB |= (1 << DDB1);

//   // Duty Cycle  (20ms period) / 1/50Hz . Duty Cycle of 5 volts = 1-2ms
//   /*
//   Clock cycles per second = Clock Speed / Prescaler = 16,000,000 / 64 = 250,000 cycles/second.
//   Clock cycles in 20 milliseconds = 250,000 cycles/second * 0.02 seconds = 5,000 cycles. (Minus 1, since the starting value is 0)
// */
//   OCR1A = 4999;  //

//   // OCR1A = 375;

//   // // Configure Timer 1 for input capture
//   //   TCCR1A = 0; // Normal mode
//   //   TCCR1B |= (1 << ICES1) | (1 << ICNC1); // Capture rising edge, noise canceler enabled
//   //   TIMSK1 |= (1 << ICIE1); // Enable input capture interrupt
// }
// void triggerUltrasonicSensor() {
//   // Serial.println("Triggering UltraSonic");
//   // Send a 10 microsecond trigger pulse on a digital output pin
//   // Clear the trigPin
//   // digitalWrite(trigPin, LOW);
//   PORTB &= ~(1 << trigPin);
//   delayMicroseconds(2);

//   // Set the trigPin on HIGH state for 10 microseconds
//   // digitalWrite(trigPin, HIGH);
//   PORTB |= (1 << trigPin);
//   delayMicroseconds(10);

//   // digitalWrite(trigPin, LOW);
//   PORTB &= ~(1 << trigPin);

//   // Enable input capture interrupt
//   // TCCR1B |= (1 << ICES1) | (1 << ICNC1) | (1 << ICIE1);  // Capture rising edge, noise canceler enabled, enable input capture interrupt
// }