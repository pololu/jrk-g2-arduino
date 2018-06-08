// This example shows how to read the scaled feedback variable
// from the Jrk G2 over serial.
//
// The sketch reads the scaled feedback and prints it to the
// serial monitor.
//
// The Jrk's input mode must be set to "Serial/I2C/USB".  The
// Jrk's device number must be set to its default value of 11.
//
// Please see https://github.com/pololu/jrk-g2-arduino for
// details on how to make the connections between the Arduino and
// the Jrk G2.

#include <JrkG2.h>

JrkG2I2C jrk;

void setup()
{
  Serial.begin(115200);

  // Set up I2C.
  Wire.begin();
}

void loop()
{
  delay(1000);
  uint16_t feedback = jrk.getScaledFeedback();
  Serial.println(feedback);
}
