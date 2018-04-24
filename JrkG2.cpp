#include <JrkG2.h>
#include <Arduino.h>

/**** JrkG2Serial ****/

void JrkG2Serial::commandW7(uint8_t cmd, uint8_t val)
{
  sendCommandHeader(cmd);
  serialW7(val);

  _lastError = 0;
}

void JrkG2Serial::commandWs14(JrkG2Command cmd, int16_t val)
{
  sendCommandHeader(cmd);
  serialW7(val); // lower 7 bits
  serialW7(val >> 7); // upper 7 bits

  _lastError = 0;
}

uint8_t JrkG2Serial::commandR8(uint8_t cmd)
{
  uint8_t val;

  sendCommandHeader(cmd);

  uint8_t byteCount = _stream->readBytes(&val, 1);
  if (byteCount != 1)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  return val;
}

uint16_t JrkG2Serial::commandR16(uint8_t cmd)
{
  uint8_t buffer[2];

  sendCommandHeader(cmd);

  uint8_t byteCount = _stream->readBytes(buffer, 2);
  if (byteCount != 2)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  return ((uint16_t)buffer[0] << 0) | ((uint16_t)buffer[1] << 8);
}

void JrkG2Serial::segmentRead(JrkG2Command cmd, uint8_t offset,
  uint8_t length, void * buffer)
{
  length &= 0x7F;
  sendCommandHeader(cmd);
  serialW7(offset);
  serialW7(length);

  uint8_t byteCount = _stream->readBytes((uint8_t *)buffer, length);
  if (byteCount != length)
  {
    _lastError = JrkG2CommReadError;

    // Set the buffer bytes to 0 so the program will not use an uninitialized
    // variable.
    memset(buffer, 0, length);
    return;
  }

  _lastError = 0;
}

void JrkG2Serial::segmentWrite(JrkG2Command cmd, uint8_t offset,
  uint8_t length, void * buffer)
{
  length &= 0x7; // max length = 7
  sendCommandHeader(cmd);
  serialW7(offset);
  serialW7(length);

  uint8_t msbs = 0; // most-significant bits
  for (uint8_t i = 0; i < length; i++)
  {
    serialW7(((uint8_t *)buffer)[i]); // first byte of buffer with MSB cleared
    msbs |= ((((uint8_t *)buffer)[i] >> 7) & 1) << i; // bit i = MSB of buffer[i]
  }
  serialW7(msbs);

  _lastError = 0;
}

void JrkG2Serial::sendCommandHeader(uint8_t cmd)
{
  if (_deviceNumber == 255)
  {
    // Compact protocol
    _stream->write((uint8_t)cmd);
  }
  else
  {
    // Pololu protocol
    _stream->write(0xAA);
    serialW7(_deviceNumber);
    serialW7((uint8_t)cmd);
  }
  _lastError = 0;
}

/**** JrkG2I2C ****/

void JrkG2I2C::commandQuick(JrkG2Command cmd)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  _lastError = Wire.endTransmission();
}

void JrkG2I2C::commandW7(uint8_t cmd, uint8_t val)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  Wire.write(val & 0x7F);
  _lastError = Wire.endTransmission();
}

void JrkG2I2C::commandWs14(JrkG2Command cmd, int16_t val)
{
  uint8_t upper = (val >> 8) & 0x3F; // truncate upper byte to 6 bits
  if (val < 0) { upper |= 0xC0; } // sign-extend

  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  Wire.write(val); // lower 8 bits
  Wire.write(upper); // upper 6 bits, sign-extended
  _lastError = Wire.endTransmission();
}

uint8_t JrkG2I2C::commandR8(uint8_t cmd)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  _lastError = Wire.endTransmission(false);  // no stop (repeated start)
  if (_lastError) { return 0; }

  uint8_t byteCount = Wire.requestFrom(_address, (uint8_t)1);
  if (byteCount != 1)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  uint8_t val = Wire.read();
  return val;
}

uint16_t JrkG2I2C::commandR16(uint8_t cmd)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  _lastError = Wire.endTransmission(false);  // no stop (repeated start)
  if (_lastError) { return 0; }

  uint8_t byteCount = Wire.requestFrom(_address, (uint8_t)2);
  if (byteCount != 2)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  uint8_t valL = Wire.read();
  uint8_t valH = Wire.read();
  return (uint16_t)valL | ((uint16_t)valH << 8);
}

void JrkG2I2C::segmentRead(JrkG2Command cmd, uint8_t offset,
  uint8_t length, void * buffer)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  Wire.write(offset);
  _lastError = Wire.endTransmission(false);  // no stop (repeated start)
  if (_lastError)
  {
    // Set the buffer bytes to 0 so the program will not use an uninitialized
    // variable.
    memset(buffer, 0, length);
    return;
  }

  uint8_t byteCount = Wire.requestFrom(_address, (uint8_t)length);
  if (byteCount != length)
  {
    _lastError = JrkG2CommReadError;
    memset(buffer, 0, length);
    return;
  }

  _lastError = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    ((uint8_t *)buffer)[i] = Wire.read();
  }
}

void JrkG2I2C::segmentWrite(JrkG2Command cmd, uint8_t offset,
  uint8_t length, void * buffer)
{
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)cmd);
  Wire.write(offset);
  Wire.write(length);
  for (uint8_t i = 0; i < length; i++)
  {
    Wire.write(((uint8_t *)buffer)[i]);
  }
  _lastError = Wire.endTransmission();
}
