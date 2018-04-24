// This example shows how to control the Jrk G2 over serial by
// sending Set Target commands.
//
// The Jrk's input mode must be set to "Serial/I2C/USB".  The
// serial mode must be set to "UART, fixed baud rate", and the
// baud rate must be set to "9600".  CRC should not be enabled.
// This example uses the Compact Protocol, so the Jrk's device
// number is not used, and can be set to anything.
//
// Please see https://github.com/pololu/jrk-g2-arduino for
// details on how to make the connections between the Arduino and
// the Jrk G2.

#include <JrkG2.h>

// On boards with a hardware serial port available for use, use
// that port to communicate with the Jrk. For other boards,
// create a SoftwareSerial object using pin 10 to receive (RX)
// and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define jrkSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial jrkSerial(10, 11);
#endif

JrkG2Serial jrk(jrkSerial);

void setup()
{
  // Set the baud rate.
  jrkSerial.begin(9600);
}

void loop()
{
  delay(1000);
  jrk.setTarget(2348);
  delay(1000);
  jrk.setTarget(1748);
}
