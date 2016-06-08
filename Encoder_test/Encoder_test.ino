/* Encoder Library - NoInterrupts Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

// If you define ENCODER_DO_NOT_USE_INTERRUPTS *before* including
// Encoder, the library will never use interrupts.  This is mainly
// useful to reduce the size of the library when you are using it
// with pins that do not support interrupts.  Without interrupts,
// your program must call the read() function rapidly, or risk
// missing changes in position.
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

//arduino uno int pin 2,3
//arduino mega int pin 2, 3, 18, 19, 20, 21

// Change these two numbers to the pins connected to your encoder.
//   With ENCODER_DO_NOT_USE_INTERRUPTS, no interrupts are ever
//   used, even if the pin has interrupt capability
Encoder myEnc(19, 42);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic NoInterrupts Test:");
}

long position  = -999;

void loop() {
  long newPos = myEnc.read();
  if (newPos != position) {
    position = newPos;
    Serial.println(position);
  }

}
