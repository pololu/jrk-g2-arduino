# Jrk G2 library for Arduino

Version: 1.0.1<br>
Release date: 2018-06-08<br>
[![Build Status](https://travis-ci.org/pololu/jrk-g2-arduino.svg?branch=master)](https://travis-ci.org/pololu/jrk-g2-arduino)<br>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with a
[Jrk G2 USB Motor Controller with Feedback][jrk]
using serial or I&sup2;C.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.8.x or later;
we have not tested it with earlier versions.  This library should support any
Arduino-compatible board, including the [Pololu A-Star controllers][a-star].

## Getting started

### Hardware

The Jrk G2 USB Motor Controllers with Feedback can be purchased from Pololu's
website.  Before continuing, careful reading of the [Jrk G2 User's Guide][guide]
is recommended.

### I2C connections

To control the Jrk G2 using I2C, you should connect your board's SCL pin to the
Jrk's SCL pin, connect your board's SDA pin to the Jrk's SDA pin, and connect
one of your board's GND pins to one of the Jrk's GND pins.

### Serial connections

To control the Jrk G2 using asynchronous TTL serial, you need to at least
connect your board's TX pin (as defined in the table below) to the Jrk's RX pin,
and connect your board's GND pin to one of the Jrk's GND pins.  If you want to
read information from the Jrk, you must also connect your board's RX pin (as
defined in the table below) to the Jrk's TX pin.

The example sketches for this library use a hardware serial port on your Arduino
if one is available: if you Arduino environment defines
`SERIAL_PORT_HARDWARE_OPEN`, the examples will use that port.  The pins for this
serial port are different depending on which Arduino you are using.

| Microcontroller Board | Hardware serial? | MCU RX pin | MCU TX pin |
|-----------------------|------------------|------------|------------|
| A-Star 32U4           |        Yes       |      0     |      1     |
| A-Star 328PB          |        Yes       |     12     |     11     |
| Arduino Leonardo      |        Yes       |      0     |      1     |
| Arduino Micro         |        Yes       |      0     |      1     |
| Arduino Mega 2560     |        Yes       |     19     |     18     |
| Arduino Due           |        Yes       |     19**   |     18     |
| Arduino Uno           |        No        |     10     |     11     |
| Arduino Yun           |        No        |     10     |     11     |

** The Due's serial port is 3.3&nbsp;V, so it should not be directly connected
to the Jrk's 5&nbsp;V TX line. A voltage divider or level shifter can be
used.

### Jrk configuration

Before using the example sketches, you should use the
Jrk G2 Configuration Utility software to apply these settings:

* Control mode: Serial/I&sup2;C/USB.
* Serial mode: UART, fixed baud rate
* Baud rate: 9600
* CRC disabled
* Device number: 11

### Software

You can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then
   "Manage Libraries...".
2. Search for "JrkG2".
3. Click the JrkG2 entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the
   [latest release archive from GitHub](https://github.com/pololu/jrk-g2-arduino/releases)
   and decompress it.
2. Rename the folder "jrk-g2-arduino-xxxx" to "JrkG2".
3. Drag the "JrkG2" folder into the "libraries" directory inside your
   Arduino sketchbook directory. You can view your sketchbook location by
   opening the "File" menu and selecting "Preferences" in the Arduino IDE. If
   there is not already a "libraries" folder in that location, you should make
   the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You can
access them from the Arduino IDE by opening the "File" menu, selecting
"Examples", and then selecting "JrkG2". If you cannot find these
examples, the library was probably installed incorrectly and you should retry
the installation instructions above.

## Classes

The main classes provided by the library are listed below:

* JrkG2Base
* JrkG2Serial
* JrkG2I2C

## Documentation

For complete documentation of this library, see [the jrk-g2-arduino documentation][doc].  If you are already on that page, then click the links in the "Classes" section above.

[a-star]: https://www.pololu.com/a-star
[doc]: https://pololu.github.io/jrk-g2-arduino/
[jrk]: https://www.pololu.com/jrk
[guide]: https://www.pololu.com/docs/0J73
[ide]: https://www.arduino.cc/en/Main/Software

## Version history

* 1.0.1 (2018-06-08): Call `Serial.begin` in the two examples that use the serial monitor.
* 1.0.0 (2018-04-24): Original release.
