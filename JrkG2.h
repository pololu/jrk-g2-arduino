// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

/// \file JrkG2.h
///
/// This is the main header file for the Jrk G2 Motor Controller library
/// for Arduino.
///
/// For more information about the library, see the main repository at:
/// https://github.com/pololu/jrk-g2-arduino

#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>

/// This is used to represent a null or missing value for some of the Jrk G2's
/// 16-bit input variables.
const uint16_t JrkG2InputNull = 0xFFFF;

/// This value is returned by getLastError() if the last communication with the
/// device resulted in an unsuccessful read (e.g. timeout or NACK).
const uint8_t JrkG2CommReadError = 50;

/// This enum defines the Jrk's error bits.  See the "Error handling" section of
/// the Jrk G2 user's guide for more information about what these errors mean.
///
/// See JrkG2Base::getErrorFlagsHalting() and JrkG2Base::getErrorFlagsOccurred().
enum class JrkG2Error
{
  AwaitingCommand     = 0,
  NoPower             = 1,
  MotorDriver         = 2,
  InputInvalid        = 3,
  InputDisconnect     = 4,
  FeedbackDisconnect  = 5,
  MaxCurrentExceeded  = 6,
  SerialSignal        = 7,
  SerialOverrun       = 8,
  SerialBufferFul     = 9,
  SerialCrc           = 10,
  SerialProtocol      = 11,
  SerialTimeout       = 12,
  Overcurrent         = 13,
};

/// This enum defines the Jrk G2 command codes which are used for its serial,
/// I2C, and USB interface.  These codes are used by the library and you should
/// not need to use them.
enum class JrkG2Command
{
  SetTarget                         = 0xC0,
  SetTargetLowResRev                = 0xE0,
  SetTargetLowResFwd                = 0xE1,
  ForceDutyCycleTarget              = 0xF2,
  ForceDutyCycle                    = 0xF4,
  MotorOff                          = 0xFF,
  GetVariable8                      = 0x80,
  GetVariable16                     = 0xA0,
  GetSettings                       = 0xE3,
  GetVariables                      = 0xE5,
  SetOverridableSettings            = 0xE6,
  GetOverridableSettings            = 0xEA,
  GetCurrentChoppingOccurrenceCount = 0xEC,
};

/// This enum defines the possible force modes blah blah TODO
///
/// See JrkG2Base::getForceMode().
enum class JrkG2ForceMode
{
  None            = 0,
  DutyCycleTarget = 1,
  DutyCycle       = 2,
};

/// This enum defines the possible causes of a full microcontroller reset for
/// the Jrk G2.
///
/// See JrkG2Base::getDeviceReset().
enum class JrkG2Reset
{
  PowerUp        = 0,
  Brownout       = 1,
  ResetLine      = 2,
  Watchdog       = 4,
  Software       = 8,
  StackOverflow  = 16,
  StackUnderflow = 32,
};

/// This enum defines the Jrk G2's control and feedback pins.
enum class JrkG2Pin
{
  SCL = 0,
  SDA = 1,
  TX  = 2,
  RX  = 3,
  RC  = 4,
  AUX = 5,
  FBA = 6,
  FBT = 7,
};

/// This enum defines the bits in the Jrk G2's Options Byte 3 register.  You
/// should not need to use this directly.  See JrkG2Base::setResetIntegral(),
/// JrkG2Base::getResetIntegral(), JrkG2Base::setCoastWhenOff(), and
/// JrkG2Base::getCoastWhenOff().
enum class JrkG2OptionsByte3
{
  ResetIntegral = 0,
  CoastWhenOff = 1,
};

/// This is a base class used to represent a connection to a Jrk G2.  This class
/// provides high-level functions for sending commands to the Jrk and reading
/// data from it.
///
/// See the subclasses of this class, JrkG2Serial and JrkG2I2C.
class JrkG2Base
{
public:
  /// Sets the target of the Jrk to a value in the range 0-4095. This can
  /// represent a target duty cycle, speed, or position depending on the
  /// feedback mode.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTarget(3000);
  /// ```
  ///
  /// This function sends a "Set target" command to the Jrk G2.  If the Control
  /// mode is set to Serial/I2C/USB, the Jrk will start driving the motor to
  /// reach the target.  If the control mode is something other than
  /// Serial, this command will be silently ignored.  FIXME
  ///
  /// See also getTarget().
  void setTarget(uint16_t target)
  {
    // lower 5 bits in command byte
    // upper 7 bits in data byte
    if (target > 4095) { target = 4095; }
    commandW7((uint8_t)JrkG2Command::SetTarget | (target & 0x1F), target >> 5);
  }

  void setTargetLowResRev(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResRev, target);
  }

  void setTargetLowResFwd(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResFwd, target);
  }

  void forceDutyCycleTarget(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycleTarget, dutyCycle);
  }

  void forceDutyCycle(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycle, dutyCycle);
  }

  /// Stops the motor abruptly without respecting the deceleration limit.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.motorOff();
  /// ```
  ///
  /// This function sends a "Motor off" command to the Jrk G2.  Besides
  /// stopping the motor and setting the current position, this command also
  /// clears the "Postion uncertain" flag, sets the "Input state" to "halt", and
  /// clears the "Input after scaling" variable.  FIXME
  ///
  /// If the control mode is something other than Serial, this command will
  /// be silently ignored.
  void motorOff()
  {
    commandQuick(JrkG2Command::MotorOff);
  }

  uint16_t getInput()
  {
    return getVar16SingleByte(VarOffset::Input);
  }

  uint16_t getTarget()
  {
    return getVar16SingleByte(VarOffset::Target);
  }

  uint16_t getFeedback()
  {
    return getVar16SingleByte(VarOffset::Feedback);
  }

  uint16_t getScaledFeedback()
  {
    return getVar16SingleByte(VarOffset::ScaledFeedback);
  }

  int16_t getIntegral()
  {
    return getVar16SingleByte(VarOffset::Integral);
  }

  int16_t getDutyCycleTarget()
  {
    return getVar16SingleByte(VarOffset::DutyCycleTarget);
  }

  int16_t getDutyCycle()
  {
    return getVar16SingleByte(VarOffset::DutyCycle);
  }

  // TODO check units
  uint8_t getCurrentLowRes()
  {
    return getVar8SingleByte(VarOffset::CurrentLowRes);
  }

  bool getPIDPeriodExceeded()
  {
    return getVar8SingleByte(VarOffset::PIDPeriodExceeded);
  }

  uint16_t getPIDPeriodCount()
  {
    return getVar16SingleByte(VarOffset::PIDPeriodCount);
  }

  /// Gets the errors that are currently stopping the motor.
  ///
  /// Each bit in the returned register represents a different error.  The bits
  /// are defined in ::JrkG2Error enum.
  ///
  /// Example usage:
  /// ```
  /// uint16_t errors = jrk.getErrorFlagsHalting();
  /// if (errors & (1 << (uint8_t)JrkG2Error::NoPower))
  /// {
  ///   // handle loss of power
  /// }
  /// ```
  uint16_t getErrorFlagsHalting()
  {
    return getVar16SingleByte(VarOffset::ErrorFlagsHalting);
  }

  /// Gets the errors that have occurred since the last time this function was called.
  ///
  /// Note that the Jrk G2 Control Center constantly clears the bits in this
  /// register, so if you are running the Jrk G2 Control Center then you will
  /// not be able to reliably detect errors with this function. TODO still true?
  ///
  /// Each bit in the returned register represents a different error.  The bits
  /// are defined in ::JrkG2Error enum.
  ///
  /// Example usage:
  /// ```
  /// uint32_t errors = jrk.getErrorsOccurred();
  /// if (errors & (1 << (uint8_t)JrkG2Error::MotorDriver))
  /// {
  ///   // handle a motor driver error
  /// }
  /// ```
  uint16_t getErrorFlagsOccurred()
  {
    return getVar16SingleByte(VarOffset::ErrorFlagsOccurred);
  }

  // TODO check
  JrkG2ForceMode getForceMode()
  {
    return (JrkG2ForceMode)(getVar8SingleByte(VarOffset::FlagByte1) & 0x03);
  }

  /// Gets the measurement of the VIN voltage, in millivolts.
  ///
  /// Example usage:
  /// ```
  /// uint16_t vin = jrk.getVinVoltage();
  /// ```
  uint16_t getVinVoltage()
  {
    return getVar16SingleByte(VarOffset::VinVoltage);
  }

  // TODO check units
  uint16_t getCurrent()
  {
    return getVar16SingleByte(VarOffset::Current);
  }

  JrkG2Reset getDeviceReset()
  {
    return (JrkG2Reset)getVar8(VarOffset::DeviceReset);
  }

  /// Gets the time since the last full reset of the Jrk's microcontroller, in
  /// milliseconds.
  ///
  /// Example usage:
  /// ```
  /// uint32_t upTime = jrk.getUpTime();
  /// ```
  ///
  /// A Reset command (reset()) does not count. TODO is this applicable?
  uint32_t getUpTime()
  {
    return getVar32(VarOffset::UpTime);
  }

  /// Gets the raw pulse width measured on the Jrk's RC input, in units of
  /// twelfths of a microsecond. TODO check units
  ///
  /// Returns JrkG2InputNull if the RC input is missing or invalid. TODO still valid?
  ///
  /// Example usage:
  /// ```
  /// uint16_t pulseWidth = jrk.getRCPulseWidth();
  /// if (pulseWidth != JrkG2InputNull && pulseWidth > 18000)
  /// {
  ///   // Pulse width is greater than 1500 microseconds.
  /// }
  /// ```
  uint16_t getRCPulseWidth()
  {
    return getVar16(VarOffset::RCPulseWidth);
  }

  uint16_t getTachometerReading()
  {
    return getVar16(VarOffset::TachometerReading);
  }

  /// Gets the analog reading from the specified pin.
  ///
  /// The reading is left-justified, so 0xFFFF represents a voltage equal to the
  /// Jrk's 5V pin (approximately 4.8 V). TODO still valid?
  ///
  /// Returns JrkG2InputNull if the analog reading is disabled or not ready or
  /// the pin is invalid.
  ///
  /// Example usage:
  /// ```
  /// uint16_t reading = getAnalogReading(JrkG2Pin::SDA);
  /// if (reading != JrkG2InputNull && reading < 32768)
  /// {
  ///   // The reading is less than about 2.4 V.
  /// }
  /// ```
  // TODO is this long enough to put in the cpp?
  uint16_t getAnalogReading(JrkG2Pin pin)
  {
    switch (pin)
    {
      case JrkG2Pin::SDA:
        return getVar16(VarOffset::AnalogReadingSDA);
      case JrkG2Pin::FBA:
        return getVar16(VarOffset::AnalogReadingFBA);
      default:
        return JrkG2InputNull;
    }
  }

  /// Gets a digital reading from the specified pin.
  ///
  /// Returns `true` for high and `false` for low.
  ///
  /// Example usage:
  /// ```
  /// if (jrk.getDigitalReading(JrkG2Pin::RC))
  /// {
  ///   // Something is driving the RC pin high.
  /// }
  /// ```
  bool getDigitalReading(JrkG2Pin pin)
  {
    uint8_t readings = getVar8(VarOffset::DigitalReadings);
    return (readings >> (uint8_t)pin) & 1;
  }

  // TODO check units
  uint16_t getRawCurrent()
  {
    return getVar16(VarOffset::RawCurrent);
  }

  uint16_t getCurrentLimitCode()
  {
    return getVar16(VarOffset::CurrentLimitCode);
  }

  int16_t getLastDutyCycle()
  {
    return getVar16(VarOffset::LastDutyCycle);
  }

  uint8_t getCurrentChoppingConsecutiveCount()
  {
    return getVar8(VarOffset::CurrentChoppingConsecutiveCount);
  }

  uint8_t getCurrentChoppingOccurrenceCount()
  {
    return commandR8(JrkG2Command::GetCurrentChoppingOccurrenceCount);
  }

  /// Gets a contiguous block of settings from the Jrk G2's EEPROM.
  ///
  /// The maximum length that can be fetched is 15 bytes.
  ///
  /// Example usage:
  /// ```
  /// // Get the Jrk's serial device number.
  /// uint8_t deviceNumber;
  /// jrk.getSetting(0x28, 1, &deviceNumber);
  /// ```
  ///
  /// This library does not attempt to interpret the settings and say what they
  /// mean.  For information on how the settings are encoded in the Jrk's
  /// EEPROM, see the Jrk G2 user's guide.
  void getSetting(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentRead(JrkG2Command::GetSettings, offset, length, buffer);
  }

  void setResetIntegral(bool reset)
  {
    uint8_t tmp = getOvrSetting8(OvrSettingOffset::OptionsByte3);
    if (reset)
    {
      tmp |= 1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral;
    }
    else
    {
      tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral);
    }
    setOvrSetting8(OvrSettingOffset::OptionsByte3, tmp);
  }

  bool getResetIntegral()
  {
    return getOvrSetting8(OvrSettingOffset::OptionsByte3) >> (uint8_t)JrkG2OptionsByte3::ResetIntegral & 1;
  }

  void setCoastWhenOff(bool coast)
  {
    uint8_t tmp = getOvrSetting8(OvrSettingOffset::OptionsByte3);
    if (coast)
    {
      tmp |= 1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff;
    }
    else
    {
      tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff);
    }
    setOvrSetting8(OvrSettingOffset::OptionsByte3, tmp);
  }

  bool getCoastWhenOff()
  {
    return getOvrSetting8(OvrSettingOffset::OptionsByte3) >> (uint8_t)JrkG2OptionsByte3::CoastWhenOff & 1;
  }

  void setProportionalCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::ProportionalMultiplier, multiplier, exponent);
  }

  uint16_t getProportionalMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::ProportionalMultiplier);
  }

  uint8_t getProportionalExponent()
  {
    return getOvrSetting8(OvrSettingOffset::ProportionalExponent);
  }

  void setIntegralCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::IntegralMultiplier, multiplier, exponent);
  }

  uint16_t getIntegralMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::IntegralMultiplier);
  }

  uint8_t getIntegralExponent()
  {
    return getOvrSetting8(OvrSettingOffset::IntegralExponent);
  }

  void setDerivativeCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::DerivativeMultiplier, multiplier, exponent);
  }

  uint16_t getDerivativeMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::DerivativeMultiplier);
  }

  uint8_t getDerivativeExponent()
  {
    return getOvrSetting8(OvrSettingOffset::DerivativeExponent);
  }

  void setPIDPeriod(uint16_t period)
  {
    setOvrSetting16(OvrSettingOffset::PIDPeriod, period);
  }

  uint16_t getPIDPeriod()
  {
    return getOvrSetting16(OvrSettingOffset::PIDPeriod);
  }

  void setIntegralLimit(uint16_t limit)
  {
    setOvrSetting16(OvrSettingOffset::IntegralLimit, limit);
  }

  uint16_t getIntegralLimit()
  {
    return getOvrSetting16(OvrSettingOffset::IntegralLimit);
  }

  void setMaxDutyCycleWhileFeedbackOutOfRange(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleWhileFeedbackOutOfRange, duty);
  }

  uint16_t getMaxDutyCycleWhileFeedbackOutOfRange()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleWhileFeedbackOutOfRange);
  }

  void setMaxAccelerationForward(uint16_t accel)
  {
    setOvrSetting16(OvrSettingOffset::MaxAccelerationForward, accel);
  }

  uint16_t getMaxAccelerationForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxAccelerationForward);
  }

  void setMaxAccelerationReverse(uint16_t accel)
  {
    setOvrSetting16(OvrSettingOffset::MaxAccelerationReverse, accel);
  }

  uint16_t getMaxAccelerationReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxAccelerationReverse);
  }

  void setMaxDecelerationForward(uint16_t decel)
  {
    setOvrSetting16(OvrSettingOffset::MaxDecelerationForward, decel);
  }

  uint16_t getMaxDecelerationForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDecelerationForward);
  }

  void setMaxDecelerationReverse(uint16_t decel)
  {
    setOvrSetting16(OvrSettingOffset::MaxDecelerationReverse, decel);
  }

  uint16_t getMaxDecelerationReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDecelerationReverse);
  }

  void setMaxDutyCycleForward(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleForward, duty);
  }

  uint16_t getMaxDutyCycleForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleForward);
  }

  void setMaxDutyCycleReverse(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleReverse, duty);
  }

  uint16_t getMaxDutyCycleReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleReverse);
  }

  void setCurrentLimitCodeForward(uint16_t code)
  {
    setOvrSetting16(OvrSettingOffset::CurrentLimitCodeForward, code);
  }

  uint16_t getCurrentLimitCodeForward()
  {
    return getOvrSetting16(OvrSettingOffset::CurrentLimitCodeForward);
  }

  void setCurrentLimitCodeReverse(uint16_t code)
  {
    setOvrSetting16(OvrSettingOffset::CurrentLimitCodeReverse, code);
  }

  uint16_t getCurrentLimitCodeReverse()
  {
    return getOvrSetting16(OvrSettingOffset::CurrentLimitCodeReverse);
  }

  void setBrakeDurationForward(uint8_t duration)
  {
    setOvrSetting8(OvrSettingOffset::BrakeDurationForward, duration);
  }

  uint8_t getBrakeDurationForward()
  {
    return getOvrSetting8(OvrSettingOffset::BrakeDurationForward);
  }

  void setBrakeDurationReverse(uint8_t duration)
  {
    setOvrSetting8(OvrSettingOffset::BrakeDurationReverse, duration);
  }

  uint8_t getBrakeDurationReverse()
  {
    return getOvrSetting8(OvrSettingOffset::BrakeDurationReverse);
  }

  void setMaxCurrentForward(uint16_t current)
  {
    setOvrSetting16(OvrSettingOffset::MaxCurrentForward, current);
  }

  uint16_t getMaxCurrentForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxCurrentForward);
  }

  void setMaxCurrentReverse(uint16_t current)
  {
    setOvrSetting16(OvrSettingOffset::MaxCurrentReverse, current);
  }

  uint16_t getMaxCurrentReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxCurrentReverse);
  }

  /// Returns 0 if the last communication with the device was successful, and
  /// non-zero if there was an error.
  uint8_t getLastError()
  {
    return _lastError;
  }

protected:
  /// Zero if the last communication with the device was successful, non-zero
  /// otherwise.
  uint8_t _lastError = 0;

private:
  enum VarOffset
  {
    Input                           = 0x00, // uint16_t
    Target                          = 0x02, // uint16_t
    Feedback                        = 0x04, // uint16_t
    ScaledFeedback                  = 0x06, // uint16_t
    Integral                        = 0x08, // int16_t
    DutyCycleTarget                 = 0x0A, // int16_t
    DutyCycle                       = 0x0C, // int16_t
    CurrentLowRes                   = 0x0E, // uint8_t
    PIDPeriodExceeded               = 0x0F, // uint8_t
    PIDPeriodCount                  = 0x10, // uint16_t
    ErrorFlagsHalting               = 0x12, // uint16_t
    ErrorFlagsOccurred              = 0x14, // uint16_t

    FlagByte1                       = 0x16, // uint8_t
    VinVoltage                      = 0x17, // uint16_t
    Current                         = 0x19, // uint16_t

    // variables above can be read with single-byte commands (GetVariable)
    // variables below must be read with segment read (GetVariables)

    DeviceReset                     = 0x1F, // uint8_t
    UpTime                          = 0x20, // uint32_t
    RCPulseWidth                    = 0x24, // uint16_t
    TachometerReading               = 0x26, // uint16_t
    AnalogReadingSDA                = 0x28, // uint16_t
    AnalogReadingFBA                = 0x2A, // uint16_t
    DigitalReadings                 = 0x2C, // uint8_t
    RawCurrent                      = 0x2D, // uint16_t
    CurrentLimitCode                = 0x2F, // uint16_t
    LastDutyCycle                   = 0x31, // int16_t
    CurrentChoppingConsecutiveCount = 0x33, // uint8_t
    CurrentChoppingOccurrenceCount  = 0x34, // uint8_t; read with dedicated command
  };

  enum OvrSettingOffset
  {
    OptionsByte3                        = 0x00, // uint8_t
    ProportionalMultiplier              = 0x01, // uint16_t
    ProportionalExponent                = 0x03, // uint8_t
    IntegralMultiplier                  = 0x04, // uint16_t
    IntegralExponent                    = 0x06, // uint8_t
    DerivativeMultiplier                = 0x07, // uint16_t
    DerivativeExponent                  = 0x09, // uint8_t
    PIDPeriod                           = 0x0a, // uint16_t
    IntegralLimit                       = 0x0c, // uint16_t
    MaxDutyCycleWhileFeedbackOutOfRange = 0x0e, // uint16_t
    MaxAccelerationForward              = 0x10, // uint16_t
    MaxAccelerationReverse              = 0x12, // uint16_t
    MaxDecelerationForward              = 0x14, // uint16_t
    MaxDecelerationReverse              = 0x16, // uint16_t
    MaxDutyCycleForward                 = 0x18, // uint16_t
    MaxDutyCycleReverse                 = 0x1a, // uint16_t
    CurrentLimitCodeForward             = 0x1c, // uint16_t
    CurrentLimitCodeReverse             = 0x1e, // uint16_t
    BrakeDurationForward                = 0x20, // uint8_t
    BrakeDurationReverse                = 0x21, // uint8_t
    MaxCurrentForward                   = 0x22, // uint16_t
    MaxCurrentReverse                   = 0x24, // uint16_t
  };

  uint8_t getVar8SingleByte(uint8_t offset)
  {
    return commandR8((uint8_t)JrkG2Command::GetVariable8 | (offset + 1));
  }

  uint16_t getVar16SingleByte(uint8_t offset)
  {
    return commandR16((uint8_t)JrkG2Command::GetVariable16 | (offset + 1));
  }

  uint8_t getVar8(uint8_t offset)
  {
    uint8_t result;
    segmentRead(JrkG2Command::GetVariables, offset, 1, &result);
    return result;
  }

  uint16_t getVar16(uint8_t offset)
  {
    uint8_t buffer[2];
    segmentRead(JrkG2Command::GetVariables, offset, 2, buffer);
    return ((uint16_t)buffer[0] << 0) | ((uint16_t)buffer[1] << 8);
  }

  uint32_t getVar32(uint8_t offset)
  {
    uint8_t buffer[4];
    segmentRead(JrkG2Command::GetVariables, offset, 4, buffer);
    return ((uint32_t)buffer[0] << 0) |
      ((uint32_t)buffer[1] << 8) |
      ((uint32_t)buffer[2] << 16) |
      ((uint32_t)buffer[3] << 24);
  }

  void setOvrSetting8(uint8_t offset, uint8_t val)
  {
    segmentWrite(JrkG2Command::SetOverridableSettings, offset, 1, &val);
  }

  void setOvrSetting16(uint8_t offset, uint16_t val)
  {
    uint8_t buffer[2] = {(uint8_t)val, (uint8_t)(val >> 8)};
    segmentWrite(JrkG2Command::SetOverridableSettings, offset, 2, buffer);
  }

  // slightly faster way to set multiplier and exponent together in one segment
  // write
  void setPIDCoefficient(uint8_t offset, uint16_t multiplier, uint8_t exponent)
  {
    uint8_t buffer[3] = {(uint8_t)multiplier, (uint8_t)(multiplier >> 8), exponent};
    segmentWrite(JrkG2Command::SetOverridableSettings, offset, 3, buffer);
  }

  uint8_t getOvrSetting8(uint8_t offset)
  {
    uint8_t result;
    segmentRead(JrkG2Command::GetOverridableSettings, offset, 1, &result);
    return result;
  }

  uint16_t getOvrSetting16(uint8_t offset)
  {
    uint8_t buffer[2];
    segmentRead(JrkG2Command::GetOverridableSettings, offset, 2, buffer);
    return ((uint16_t)buffer[0] << 0) | ((uint16_t)buffer[1] << 8);
  }

  virtual void commandQuick(JrkG2Command cmd) = 0;
  virtual void commandW7(uint8_t cmd, uint8_t val) = 0;
  void commandW7(JrkG2Command cmd, uint8_t val) { commandW7((uint8_t)cmd, val); }
  virtual void commandWs14(JrkG2Command cmd, int16_t val) = 0;
  virtual uint8_t commandR8(uint8_t cmd) = 0;
  uint8_t commandR8(JrkG2Command cmd) { return commandR8((uint8_t)cmd); }
  virtual uint16_t commandR16(uint8_t cmd) = 0;
  uint16_t commandR16(JrkG2Command cmd) { return commandR16((uint8_t)cmd); }
  virtual void segmentRead(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer) = 0;
  virtual void segmentWrite(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer) = 0;
};

/// Represents a serial connection to a Jrk G2.
///
/// For the high-level commands you can use on this object, see JrkG2Base.
class JrkG2Serial : public JrkG2Base
{
public:
  /// Creates a new JrkG2Serial object.
  ///
  /// The `stream` argument should be a hardware or software serial object.
  /// This class will store a pointer to it and use it to communicate with the
  /// Jrk.  You should initialize it and set it to use the correct baud rate
  /// before sending commands with this class.
  ///
  /// The `deviceNumber` argument is optional.  If it is omitted or 255, the
  /// JrkG2Serial object will use the compact protocol.  If it is a number
  /// between 0 and 127, it specifies the device number to use in Pololu
  /// protocol, allowing you to control multiple Jrk controllers on a single
  /// serial bus.
  ///
  /// For example, to use the first open hardware serial port to send compact
  /// protocol commands to one Jrk, write this at the top of your sketch:
  /// ```
  /// JrkG2Serial jrk(SERIAL_PORT_HARDWARE_OPEN);
  /// ```
  ///
  /// For example, to use a SoftwareSerial port and send Pololu protocol
  /// commands to two different Jrk G2 controllers, write this at the top of
  /// your sketch:
  ///
  /// ```
  /// #include <SoftwareSerial.h>
  /// SoftwareSerial jrkG2Serial(10, 11);
  /// JrkG2Serial jrk1(jrkG2Serial, 14);
  /// JrkG2Serial jrk2(jrkG2Serial, 15);
  /// ```
  JrkG2Serial(Stream & stream, uint8_t deviceNumber = 255) :
    _stream(&stream),
    _deviceNumber(deviceNumber)
  {
  }

  /// Gets the serial device number specified in the constructor.
  uint8_t getDeviceNumber() { return _deviceNumber; }

private:
  Stream * const _stream;
  const uint8_t _deviceNumber;

  void commandQuick(JrkG2Command cmd) { sendCommandHeader(cmd); }
  void commandW7(uint8_t cmd, uint8_t val);
  void commandWs14(JrkG2Command cmd, int16_t val);
  uint8_t commandR8(uint8_t cmd);
  uint16_t commandR16(uint8_t cmd);
  void segmentRead(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer);
  void segmentWrite(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer);

  void sendCommandHeader(uint8_t cmd);
  void sendCommandHeader(JrkG2Command cmd) { sendCommandHeader((uint8_t)cmd); }
  void serialW7(uint8_t val) { _stream->write(val & 0x7F); }
};

/// Represents an I2C connection to a Jrk G2.
///
/// For the high-level commands you can use on this object, see JrkG2Base.
class JrkG2I2C : public JrkG2Base
{
public:
  /// Creates a new JrkG2I2C object that will use the `Wire` object to
  /// communicate with the Jrk over I2C.
  ///
  /// The `address` parameter specifies the 7-bit I2C address to use, and it
  /// must match the Jrk's "Device number" setting.  It defaults to 11.
  JrkG2I2C(uint8_t address = 11) : _address(address)
  {
  }

  // TODO: support Wire1 on Arduino Due, and bit-banging I2C on any board?

  /// Gets the I2C address specified in the constructor.
  uint8_t getAddress() { return _address; }

private:
  const uint8_t _address;

  void commandQuick(JrkG2Command cmd);
  void commandW7(uint8_t cmd, uint8_t val);
  void commandWs14(JrkG2Command cmd, int16_t val);
  uint8_t commandR8(uint8_t cmd);
  uint16_t commandR16(uint8_t cmd);
  void segmentRead(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer);
  void segmentWrite(JrkG2Command cmd, uint8_t offset,
    uint8_t length, void * buffer) ;

  void delayAfterRead();
};
