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

/// This enum defines the modes in which the Jrk G2's duty cycle target or duty
/// cycle, normally derived from the output of its PID algorithm, can be
/// overridden with a forced value.
///
/// See JrkG2Base::getForceMode(), JrkG2Base::forceDutyCycleTarget(), and
/// JrkG2Base::forceDutyCycle,
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
  /// This function sends a "Set target" command to the Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  ///
  /// See also setTargetLowResRev(), setTargetLowResFwd(), and getTarget().
  void setTarget(uint16_t target)
  {
    // lower 5 bits in command byte
    // upper 7 bits in data byte
    if (target > 4095) { target = 4095; }
    commandW7((uint8_t)JrkG2Command::SetTarget | (target & 0x1F), target >> 5);
  }

  /// Sets the target of the Jrk to a value in the range 0-127.  If the value is
  /// zero, then this command is equivalent to the Motor Off command. Otherwise,
  /// the value maps to a 12-bit target less than 2048.
  ///
  /// If the feedback mode is Analog or Tachometer, then the formula is
  /// Target = 2048 - 16 * value.
  ///
  /// If the feedback mode is None (speed control mode), then the formula is
  /// Target = 2048 - (600 / 127) * magnitude.  This means that a magnitude of
  /// 127 will set the duty cycle target to full-speed reverse (-600).
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTargetLowResRev(100);
  /// ```
  ///
  /// This function sends a "Set target Low Resolution Reverse" command to the
  /// Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  ///
  /// See also setTargetLowResFwd(), setTarget(), getTarget(), and motorOff().
  void setTargetLowResRev(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResRev, target);
  }

  /// Sets the target of the Jrk to a value in the range 0-127 that maps to a
  /// 12-bit target of 2048 or greater.
  ///
  /// If the feedback mode is Analog or Tachometer, then the formula is
  /// Target = 2048 + 16 * value.
  ///
  /// If the feedback mode is None (speed control mode), then the formula is
  /// Target = 2048 + (600 / 127) * magnitude.  This means that a magnitude of
  /// 127 will set the duty cycle target to full-speed reverse (-600), while a
  /// magnitude of zero will make the motor stop.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTargetLowResFwd(100);
  /// ```
  ///
  /// This function sends a "Set target Low Resolution Forward" command to the
  /// Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  ///
  /// See also setTargetLowResRev(), setTarget(), and getTarget().
  void setTargetLowResFwd(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResFwd, target);
  }

  /// Forces the duty cycle target of the Jrk to a value in the range -600 to
  /// +600.  This overrides the duty cycle target produced by the Jrk's PID
  /// algorithm (or a duty cycle target from a Set Target command if the
  /// feedback mode is None), but the Jrk's actual duty cycle output will still
  /// be subject to acceleration and deceleration limits. TODO: how about duty cycle limit?
  ///
  /// TODO: talk about force mode?
  ///
  /// Example usage:
  /// ```
  /// jrkG2.forceDutyCycleTarget(250);
  /// ```
  ///
  /// This function sends a "Force Duty Cycle Target" command to the Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  ///
  /// See also forceDutyCycle() and getForceMode().
  void forceDutyCycleTarget(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycleTarget, dutyCycle);
  }

  /// Forces the duty cycle of the Jrk to a value in the range -600 to +600.
  /// This overrides the duty cycle target produced by the Jrk's PID
  /// algorithm (or a duty cycle target from a Set Target command if the
  /// feedback mode is None) and ignores the acceleration and deceleration
  /// limits. TODO: how about duty cycle limit?
  ///
  /// TODO: talk about force mode?
  ///
  /// Example usage:
  /// ```
  /// jrkG2.forceDutyCycle(250);
  /// ```
  ///
  /// This function sends a "Force Duty Cycle" command to the Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  ///
  /// See also forceDutyCycleTarget() and getForceMode().
  void forceDutyCycle(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycle, dutyCycle);
  }

  /// Turns the motor off.  The Jrk can be configured to either brake or coast
  /// while the motor is off. TODO: does deceleration apply?
  ///
  /// Example usage:
  /// ```
  /// jrkG2.motorOff();
  /// ```
  ///
  /// This function sends a "Motor off" command to the Jrk G2.  TODO: error flags?
  ///
  /// If the input mode is not Serial, or if the command is received on a serial
  /// interface other than the selected one (USB or UART), this command will be
  /// silently ignored.
  void motorOff()
  {
    commandQuick(JrkG2Command::MotorOff);
  }

  /// Gets the raw, un-scaled input value, representing a measurement taken by
  /// the Jrk of the input to the system.  In serial input mode, the input is
  /// equal to the target, which can be set to any value 0-4095 using serial
  /// commands.  In analog input mode, the input is a measurement of the voltage
  /// on the RX pin, where 0 is 0 V and 4092 is a voltage equal to the Jrk's 5V
  /// pin (approximately 4.8 V).  In pulse width input mode, the input is the
  /// duration of the last pulse measured, in units of 2/3 us. TODO: check 4092
  ///
  /// See the Jrk G2 user's guide for more information about input modes.
  ///
  /// See also getTarget() and setTarget().
  uint16_t getInput()
  {
    return getVar16SingleByte(VarOffset::Input);
  }

  /// Gets the Jrk's target.  In serial input mode, the input is set directly
  /// with serial commands.  In the other input modes, the target is computed by
  /// scaling the input.  The input scaling can be configured.
  ///
  /// See also setTarget() and getInput().
  uint16_t getTarget()
  {
    return getVar16SingleByte(VarOffset::Target);
  }

  /// Gets the raw, un-scaled feedback value, representing a measurement taken
  /// by the Jrk of the output of the system. In analog feedback mode, the
  /// feedback is a measurement of the voltage on the FBA pin, where 0 is 0 V
  /// and 4092 is a voltage equal to the Jrk's 5V pin (approximately 4.8 V).
  /// In no feedback mode (speed control mode), the feedback is always zero.  TODO: check 4092
  ///
  /// See also getScaledFeedback().
  uint16_t getFeedback()
  {
    return getVar16SingleByte(VarOffset::Feedback);
  }

  /// Gets the Jrk's scaled feedback value.  The feedback scaling can be
  /// configured.
  ///
  /// See also getFeedback().
  uint16_t getScaledFeedback()
  {
    return getVar16SingleByte(VarOffset::ScaledFeedback);
  }

  /// Gets the Jrk's error sum (integral).  Every PID period, the error (scaled
  /// feedback minus target) is added to the error sum.  The error sum gets
  /// reset to zero whenever the Jrk is not driving the motor, and can
  /// optionally be reset whenever the proportional term of the PID calculation
  /// exceeds the maximum duty cycle. There is also a configurable integral
  /// limit that the integral can not exceed.
  int16_t getIntegral()
  {
    return getVar16SingleByte(VarOffset::Integral);
  }

  /// Gets the Jrk's duty cycle target, which is the duty cycle that it is
  /// trying to achieve.  A value of -600 or less means full speed reverse,
  /// while a value of 600 or more means full speed forward.  A value of 0 means
  /// braking.  In no feedback mode (speed control mode), the duty cycle target
  /// is normally the target minus 2048. In other feedback modes, the duty cycle
  /// target is normally the sum of the proportional, integral, and derivative
  /// terms of the PID algorithm.  In any mode, the duty cycle target can be
  /// overridden with a Force Duty Cycle Target command.  TODO: check "or less"/"more"
  ///
  /// See also getDutyCycle(), getLastDutyCycle(), and forceDutyCycleTarget().
  int16_t getDutyCycleTarget()
  {
    return getVar16SingleByte(VarOffset::DutyCycleTarget);
  }

  /// Gets the duty cycle the Jrk is driving the motor with.  A value of -600 or less means full speed reverse,
  /// while a value of 600 or more means full speed forward.  A value of 0 means
  /// braking.  The duty cycle could be different from the duty cycle target
  /// because it normally takes into account the Jrk's configurable limits for
  /// maximum acceleration, maximum deceleration, maximum duty cycle, maximum
  /// current, and brake duration.  The duty cycle can be overridden with a
  /// Force Duty Cycle command.  TODO: check "or less"/"more"; does max current limit apply?
  ///
  /// See also getLastDutyCycle(), getDutyCycleTarget(), and forceDutyCycle().
  int16_t getDutyCycle()
  {
    return getVar16SingleByte(VarOffset::DutyCycle);
  }

  /// Gets a low-resolution (TODO range) representation of the Jrk's measurement
  /// of the current running through the motor.  TODO does current limit need to be enabled?
  ///
  /// See also getCurrent().
  uint8_t getCurrentLowRes()
  {
    return getVar8SingleByte(VarOffset::CurrentLowRes);
  }

  /// Returns true if the Jrk's most recent PID cycle took more time than the
  /// configured PID period.  This indicates that the Jrk does not have time to
  /// perform all of its tasks at the desired rate.  Most often, this is caused
  /// by the requested number of analog samples for input or feedback being
  /// too high for the configured PID period. TODO check/simplify
  bool getPIDPeriodExceeded()
  {
    return getVar8SingleByte(VarOffset::PIDPeriodExceeded);
  }

  /// Returns the number of PID periods that have elapsed.  It resets to 0 after
  /// reaching 65535.  The duration of the PID period can be configured.
  uint16_t getPIDPeriodCount()
  {
    return getVar16SingleByte(VarOffset::PIDPeriodCount);
  }

  /// Gets the errors that are currently stopping the motor and clears any
  /// latched errors that are enabled.  Calling this function is equivalent to
  /// reading the "Currently stopping motor?" column in the Errors tab of the
  /// configuration utility, and then clicking the "Clear Errors" button.
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
  ///
  /// See also getErrorFlagsOccurred().
  uint16_t getErrorFlagsHalting()
  {
    return getVar16SingleByte(VarOffset::ErrorFlagsHalting);
  }

  /// Gets the errors that have occurred since the last time this function was
  /// called.  Unlike getErrorFlagsHalting(), calling this function has no
  /// effect on the motor.
  ///
  /// Note that the Jrk G2 Control Center constantly clears the bits in this
  /// register, so if you are running the Jrk G2 Control Center then you will
  /// not be able to reliably detect errors with this function.
  ///
  /// Each bit in the returned register represents a different error.  The bits
  /// are defined in the ::JrkG2Error enum.
  ///
  /// Example usage:
  /// ```
  /// uint32_t errors = jrk.getErrorsOccurred();
  /// if (errors & (1 << (uint8_t)JrkG2Error::MotorDriver))
  /// {
  ///   // handle a motor driver error
  /// }
  /// ```
  ///
  /// See also getErrorFlagsHalting().
  uint16_t getErrorFlagsOccurred()
  {
    return getVar16SingleByte(VarOffset::ErrorFlagsOccurred);
  }

  /// Returns an indication of whether the Jrk's duty cycle target or duty cycle
  /// are being overridden with a forced value.
  ///
  /// Example usage:
  /// ```
  /// if (jrk.getForceMode() == JrkG2ForceMode::DutyCycleTarget)
  /// {
  ///   // The duty cycle target is being overridden with a forced value.
  /// }
  /// ```

  /// See also forceDutyCycleTarget() and forceDutyCycle().
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

  /// Gets the Jrk's measurement of the current running through the motor, in
  /// milliamps.  TODO right units? does current limit need to be enabled?
  ///
  /// See also getCurrentLowRes().
  uint16_t getCurrent()
  {
    return getVar16SingleByte(VarOffset::Current);
  }

  /// Gets the cause of the Jrk's last full microcontroller reset.
  ///
  /// Example usage:
  /// ```
  /// if (jrk.getDeviceReset() == JrkG2Reset::Brownout)
  /// {
  ///   // There was a brownout reset; the power supply could not keep up.
  /// }
  /// ```
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
  uint32_t getUpTime()
  {
    return getVar32(VarOffset::UpTime);
  }

  /// Gets the raw pulse width measured on the Jrk's RC input, in units of
  /// twelfths of a microsecond.  TODO check units
  ///
  /// Returns JrkG2InputNull if the RC input is missing or invalid.  TODO still valid?
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

  /// Gets the pulse rate measured on the Jrk's FBT (tachometer feedback) pin,
  /// in units of pulses per PID period. TODO check
  ///
  /// Example usage:
  /// ```
  /// uint16_t tachReading = jrk.getTachometerReading();
  /// if (tachReading > 10)
  /// {
  ///   // Tachometer pulse rate is greater than 10 pulses per PID period.
  /// }
  /// ```
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

  /// Gets the Jrk's raw measurement of the current running through the motor.  TODO check/fix
  ///
  /// See also getCurrentLowRes().
  uint16_t getRawCurrent()
  {
    return getVar16(VarOffset::RawCurrent);
  }

  /// Gets the Jrk's raw measurement of the current running through the motor.  TODO check/fix
  ///
  /// See also getCurrentLowRes().
  uint16_t getCurrentLimitCode()
  {
    return getVar16(VarOffset::CurrentLimitCode);
  }

  /// Gets the duty cycle the Jrk drove the motor with in the last PID period.  TODO check/fix
  ///
  /// See also getDutyCycle() and getDutyCycleTarget().
  int16_t getLastDutyCycle()
  {
    return getVar16(VarOffset::LastDutyCycle);
  }

  /// Gets the number of consecutive PID periods during which current chopping
  /// has been active.  TODO check/fix
  ///
  /// See also getCurrentChoppingOccurrenceCount().
  uint8_t getCurrentChoppingConsecutiveCount()
  {
    return getVar8(VarOffset::CurrentChoppingConsecutiveCount);
  }

  /// Gets the number of PID periods during which current chopping has been
  /// active since the last time this function was called.  TODO check/fix
  ///
  /// See also getCurrentChoppingConsecutiveCount().
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
  /// EEPROM, see the Jrk G2 user's guide. TODO this might change
  void getSetting(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentRead(JrkG2Command::GetSettings, offset, length, buffer);
  }

  /// Sets or clears the option to reset the Jrk's error sum (integral) when
  /// the proportional term exceeds the maximum duty cycle.  When enabled, this
  /// can help limit integral wind-up, or the uncontrolled growth of the
  /// integral when the feedback system is temporarily unable to keep the error
  /// small. This might happen, for example, when the target is changing
  /// quickly.
  ///
  /// See also getResetIntegral().
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

  /// Returns true if the option is enabled to reset the Jrk's integral when
  /// the proportional term exceeds the maximum duty cycle.
  ///
  /// See also getResetIntegral().
  bool getResetIntegral()
  {
    return getOvrSetting8(OvrSettingOffset::OptionsByte3) >> (uint8_t)JrkG2OptionsByte3::ResetIntegral & 1;
  }

  /// Sets or clears the option for the Jrk's motor driver to coast when the
  /// motor is off. If this option is not set, the motor will instead brake when
  /// off.
  ///
  /// See also getCoastWhenOff().
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

  /// Returns true if the option is enabled for the Jrk's motor driver to coast
  /// when the motor is off.
  ///
  /// See also setCoastWhenOff().
  bool getCoastWhenOff()
  {
    return getOvrSetting8(OvrSettingOffset::OptionsByte3) >> (uint8_t)JrkG2OptionsByte3::CoastWhenOff & 1;
  }

  /// Sets the coefficient for the proportional term of the Jrk's PID algorithm.
  /// This term is proportional to the error (the difference between the scaled
  /// feedback and target).  The coefficient takes the form
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0-1023, and the exponent can range from
  /// 0-18.
  ///
  /// Example usage:
  /// ```
  /// // Set the proportional coefficient to 1.125 (9/(2^3)).
  /// jrk.setProportionalCoefficient(9, 3);
  /// ```
  ///
  /// See also getProportionalMultiplier() and getProportionalExponent(), as
  /// well as setIntegralCoefficient() and setDerivativeCoefficient().
  void setProportionalCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::ProportionalMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the coefficient for the proportional term of
  /// the Jrk's PID algorithm.
  ///
  /// See also getProportionalExponent() and setProportionalCoefficient().
  uint16_t getProportionalMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::ProportionalMultiplier);
  }

  /// Gets the exponent part of the coefficient for the proportional term of the
  /// Jrk's PID algorithm.
  ///
  /// See also getProportionalMultiplier() and setProportionalCoefficient().
  uint8_t getProportionalExponent()
  {
    return getOvrSetting8(OvrSettingOffset::ProportionalExponent);
  }

  /// Sets the coefficient for the integral term of the Jrk's PID algorithm.
  /// This term is proportional to the accumulated sum of the error over time.
  /// The coefficient takes the form
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0-1023, and the exponent can range from
  /// 0-18.
  ///
  /// See also getIntegralMultiplier() and getIntegralExponent(), as
  /// well as setProportionalCoefficient() and setDerivativeCoefficient().
  void setIntegralCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::IntegralMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the coefficient for the integral term of the
  /// Jrk's PID algorithm.
  ///
  /// See also getIntegralExponent() and setIntegralCoefficient().
  uint16_t getIntegralMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::IntegralMultiplier);
  }

  /// Gets the exponent part of the coefficient for the integral term of the
  /// Jrk's PID algorithm.
  ///
  /// See also getIntegralMultiplier() and setIntegralCoefficient().
  uint8_t getIntegralExponent()
  {
    return getOvrSetting8(OvrSettingOffset::IntegralExponent);
  }

  /// Sets the coefficient for the integral term of the Jrk's PID algorithm.
  /// This term is proportional to the difference of the error relative to the
  /// previous PID period (or the error's rate of change).  The coefficient
  /// takes the form
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0-1023, and the exponent can range from
  /// 0-18.
  ///
  /// See also getDerivativeMultiplier() and getDerivativeExponent(), as
  /// well as setProportionalCoefficient() and setIntegralCoefficient().
  void setDerivativeCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(OvrSettingOffset::DerivativeMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the coefficient for the derivative term of the
  /// Jrk's PID algorithm.
  ///
  /// See also getDerivativeExponent() and setDerivativeCoefficient().
  uint16_t getDerivativeMultiplier()
  {
    return getOvrSetting16(OvrSettingOffset::DerivativeMultiplier);
  }

  /// Gets the exponent part of the coefficient for the derivative term of the
  /// Jrk's PID algorithm.
  ///
  /// See also getDerivativeMultiplier() and setDerivativeCoefficient().
  uint8_t getDerivativeExponent()
  {
    return getOvrSetting8(OvrSettingOffset::DerivativeExponent);
  }

  /// Sets the PID period of the Jrk, which is the rate at which it runs through
  /// all of its calculations, in milliseconds.  Note that a higher PID period
  /// will result in a more slowly changing integral and a higher derivative, so
  /// the two corresponding PID coefficients might need to be adjusted whenever
  /// the PID period is changed.
  ///
  /// See also getPIDPeriod().
  void setPIDPeriod(uint16_t period)
  {
    setOvrSetting16(OvrSettingOffset::PIDPeriod, period);
  }

  /// Gets the PID period of the Jrk, in milliseconds.
  ///
  /// See also setPIDPeriod().
  uint16_t getPIDPeriod()
  {
    return getOvrSetting16(OvrSettingOffset::PIDPeriod);
  }

  /// Sets the Jrk's integral limit, which limits the magnitude of the integral.
  /// This can help limit integral wind-up.  The limit can range from 0-32767.
  ///
  /// Note that the maximum value of the integral term can be computed as the
  /// integral coefficient times the integral limit: if this is very small
  /// compared to 600 (maximum duty cycle), the integral term will have at most
  /// a very small effect on the duty cycle.
  ///
  /// See also getIntegralLimit().
  void setIntegralLimit(uint16_t limit)
  {
    setOvrSetting16(OvrSettingOffset::IntegralLimit, limit);
  }

  /// Gets the Jrk's integral limit.
  ///
  /// See also setIntegralLimit().
  uint16_t getIntegralLimit()
  {
    return getOvrSetting16(OvrSettingOffset::IntegralLimit);
  }

  /// Sets the Jrk's maximum duty cycle while its feedback is out of range.
  /// This is an option to limit possible damage to systems by reducing the
  /// maximum duty cycle whenever the feedback value is beyond the absolute
  /// minimum and maximum values. This can be used, for example, to slowly bring
  /// a system back into its valid range of operation when it is dangerously
  /// near a limit. The Feedback disconnect error should be disabled when this
  /// option is used.
  ///
  /// See also getMaxDutyCycleWhileFeedbackOutOfRange().
  void setMaxDutyCycleWhileFeedbackOutOfRange(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleWhileFeedbackOutOfRange, duty);
  }

  /// Sets the Jrk's maximum duty cycle while its feedback is out of range.
  ///
  /// See also setMaxDutyCycleWhileFeedbackOutOfRange().
  uint16_t getMaxDutyCycleWhileFeedbackOutOfRange()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleWhileFeedbackOutOfRange);
  }

  /// Sets the Jrk's maximum acceleration in the forward direction.  This is the
  /// maximum amount in a single PID period that the the duty cycle can increase
  /// by in the forward direction.
  ///
  /// See also getMaxAccelerationForward(), setMaxAccelerationReverse(), and
  /// setMaxDecelerationForward().
  void setMaxAccelerationForward(uint16_t accel)
  {
    setOvrSetting16(OvrSettingOffset::MaxAccelerationForward, accel);
  }

  /// Gets the Jrk's maximum acceleration in the forward direction.
  ///
  /// See also setMaxAccelerationForward(), getMaxAccelerationReverse(), and
  /// getMaxDecelerationForward().
  uint16_t getMaxAccelerationForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxAccelerationForward);
  }

  /// Sets the Jrk's maximum acceleration in the reverse direction.  This is the
  /// maximum amount in a single PID period that the the duty cycle can increase
  /// by in the reverse direction.
  ///
  /// See also getMaxAccelerationReverse(), setMaxAccelerationForward(), and
  /// setMaxDecelerationReverse().
  void setMaxAccelerationReverse(uint16_t accel)
  {
    setOvrSetting16(OvrSettingOffset::MaxAccelerationReverse, accel);
  }

  /// Gets the Jrk's maximum acceleration in the reverse direction.
  ///
  /// See also setMaxAccelerationReverse(), getMaxAccelerationForward(), and
  /// getMaxDecelerationReverse().
  uint16_t getMaxAccelerationReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxAccelerationReverse);
  }

  /// Sets the Jrk's maximum accelerations in both directions.
  void setMaxAccelerations(uint16_t forwardAccel, uint16_t reverseAccel)
  {
    setOvrSetting16x2(OvrSettingOffset::MaxAccelerationForward, forwardAccel, reverseAccel);
  }

  /// Sets the Jrk's maximum deceleration in the forward direction.  This is the
  /// maximum amount in a single PID period that the the duty cycle can decrease
  /// by in the forward direction.
  ///
  /// See also getMaxDecelerationForward(), setMaxDecelerationReverse(), and
  /// setMaxAccelerationForward().
  void setMaxDecelerationForward(uint16_t decel)
  {
    setOvrSetting16(OvrSettingOffset::MaxDecelerationForward, decel);
  }

  /// Gets the Jrk's maximum deceleration in the forward direction.
  ///
  /// See also getMaxDecelerationForward(), setMaxDecelerationReverse(), and
  /// setMaxAccelerationForward().
  uint16_t getMaxDecelerationForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDecelerationForward);
  }

  /// Sets the Jrk's maximum deceleration in the reverse direction.  This is the
  /// maximum amount in a single PID period that the the duty cycle can decrease
  /// by in the reverse direction.
  ///
  /// See also getMaxDecelerationReverse(), setMaxDecelerationForward(), and
  /// setMaxAccelerationReverse().
  void setMaxDecelerationReverse(uint16_t decel)
  {
    setOvrSetting16(OvrSettingOffset::MaxDecelerationReverse, decel);
  }

  /// Gets the Jrk's maximum deceleration in the reverse direction.
  ///
  /// See also setMaxDecelerationReverse(), getMaxDecelerationForward(), and
  /// getMaxAccelerationReverse().
  uint16_t getMaxDecelerationReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDecelerationReverse);
  }

  /// Sets the Jrk's maximum decelerations in both directions.
  void setMaxDecelerations(uint16_t forwardDecel, uint16_t reverseDecel)
  {
    setOvrSetting16x2(OvrSettingOffset::MaxDecelerationForward, forwardDecel, reverseDecel);
  }

  /// Sets the Jrk's maximum duty cycle in the forward direction.
  ///
  /// See also getMaxDutyCycleForward() and setMaxDutyCycleReverse().
  void setMaxDutyCycleForward(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleForward, duty);
  }

  /// Gets the Jrk's maximum duty cycle in the forward direction.
  ///
  /// See also setMaxDutyCycleForward() and getMaxDutyCycleReverse().
  uint16_t getMaxDutyCycleForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleForward);
  }

  /// Sets the Jrk's maximum duty cycle in the reverse direction.
  ///
  /// See also getMaxDutyCycleReverse() and setMaxDutyCycleForward().
  void setMaxDutyCycleReverse(uint16_t duty)
  {
    setOvrSetting16(OvrSettingOffset::MaxDutyCycleReverse, duty);
  }

  /// Gets the Jrk's maximum duty cycle in the reverse direction.
  ///
  /// See also setMaxDutyCycleReverse() and getMaxDutyCycleForward().
  uint16_t getMaxDutyCycleReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxDutyCycleReverse);
  }

  /// Sets the Jrk's maximum duty cycles in both directions.
  void setMaxDutyCycles(uint16_t forwardDuty, uint16_t reverseDuty)
  {
    setOvrSetting16x2(OvrSettingOffset::MaxDutyCycleForward, forwardDuty, reverseDuty);
  }

  /// Sets the Jrk's current limit code for driving in the forward direction.  TODO check/fix
  ///
  /// See also getCurrentLimitCodeForward() and setCurrentLimitCodeReverse().
  void setCurrentLimitCodeForward(uint16_t code)
  {
    setOvrSetting16(OvrSettingOffset::CurrentLimitCodeForward, code);
  }

  /// Gets the Jrk's current limit code for driving in the forward direction.  TODO check/fix
  ///
  /// See also setCurrentLimitCodeForward() and getCurrentLimitCodeReverse().
  uint16_t getCurrentLimitCodeForward()
  {
    return getOvrSetting16(OvrSettingOffset::CurrentLimitCodeForward);
  }

  /// Sets the Jrk's current limit code for driving in the reverse direction.  TODO check/fix
  ///
  /// See also getCurrentLimitCodeReverse() and setCurrentLimitCodeForward().
  void setCurrentLimitCodeReverse(uint16_t code)
  {
    setOvrSetting16(OvrSettingOffset::CurrentLimitCodeReverse, code);
  }

  /// Gets the Jrk's current limit code for driving in the reverse direction.  TODO check/fix
  ///
  /// See also setCurrentLimitCodeReverse() and getCurrentLimitCodeForward().
  uint16_t getCurrentLimitCodeReverse()
  {
    return getOvrSetting16(OvrSettingOffset::CurrentLimitCodeReverse);
  }

  /// Sets the Jrk's current limit codes for driving in both directions.
  void setCurrentLimitCodes(uint16_t forwardCode, uint16_t reverseCode)
  {
    setOvrSetting16x2(OvrSettingOffset::CurrentLimitCodeForward, forwardCode, reverseCode);
  }

  /// Sets the Jrk's brake duration when switching from forward to reverse.  The
  /// Jrk will keep the motor at a duty cycle 0 for the specified time when
  /// switching directions.  This feature is most useful for large motors with
  /// high-inertia loads used with frequency feedback or speed control mode (no
  /// feedback).
  ///
  /// See also getBrakeDurationForward() and setBrakeDurationReverse().
  void setBrakeDurationForward(uint8_t duration)
  {
    setOvrSetting8(OvrSettingOffset::BrakeDurationForward, duration);
  }

  /// Gets the Jrk's brake duration when switching from forward to reverse.
  ///
  /// See also setBrakeDurationForward() and getBrakeDurationReverse().
  uint8_t getBrakeDurationForward()
  {
    return getOvrSetting8(OvrSettingOffset::BrakeDurationForward);
  }

  /// Sets the Jrk's brake duration when switching from reverse to forward.
  ///
  /// See also getBrakeDurationReverse() and setBrakeDurationForward().
  void setBrakeDurationReverse(uint8_t duration)
  {
    setOvrSetting8(OvrSettingOffset::BrakeDurationReverse, duration);
  }

  /// Gets the Jrk's brake duration when switching from reverse to forward.
  ///
  /// See also setBrakeDurationReverse() and getBrakeDurationForward().
  uint8_t getBrakeDurationReverse()
  {
    return getOvrSetting8(OvrSettingOffset::BrakeDurationReverse);
  }

  /// Sets the Jrk's brake durations for driving in both directions.
  void setBrakeDurations(uint8_t forwardDuration, uint8_t reverseDuration)
  {
    setOvrSetting8x2(OvrSettingOffset::BrakeDurationForward, forwardDuration, reverseDuration);
  }

  /// Sets the Jrk's current limit when driving in the forward direction.  The
  /// Jrk will adjust the duty cycle as necessary to limit the current to the
  /// specified value.  For accurate current limiting, acceleration should be
  /// limited; otherwise the duty cycle will tend to oscillate when the maximum
  /// current is exceeded.  TODO check
  ///
  /// See also getMaxCurrentForward() and setMaxCurrentReverse().
  void setMaxCurrentForward(uint16_t current)
  {
    setOvrSetting16(OvrSettingOffset::MaxCurrentForward, current);
  }

  /// Gets the Jrk's current limit when driving in the forward direction.
  ///
  /// See also setMaxCurrentForward() and getMaxCurrentReverse().
  uint16_t getMaxCurrentForward()
  {
    return getOvrSetting16(OvrSettingOffset::MaxCurrentForward);
  }

  /// Sets the Jrk's current limit when driving in the reverse direction.
  ///
  /// See also getMaxCurrentReverse() and setMaxCurrentForward().
  void setMaxCurrentReverse(uint16_t current)
  {
    setOvrSetting16(OvrSettingOffset::MaxCurrentReverse, current);
  }

  /// Gets the Jrk's current limit when driving in the reverse direction.
  ///
  /// See also setMaxCurrentReverse() and getMaxCurrentForward().
  uint16_t getMaxCurrentReverse()
  {
    return getOvrSetting16(OvrSettingOffset::MaxCurrentReverse);
  }

  /// Sets the Jrk's current limits for driving in both directions.
  void setMaxCurrents(uint16_t forwardCurrent, uint16_t reverseCurrent)
  {
    setOvrSetting16x2(OvrSettingOffset::MaxCurrentForward, forwardCurrent, reverseCurrent);
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

  void setOvrSetting8x2(uint8_t offset, uint8_t val1, uint8_t val2)
  {
    uint8_t buffer[2] = {val1, val2};
    segmentWrite(JrkG2Command::SetOverridableSettings, offset, 2, buffer);
  }

  void setOvrSetting16x2(uint8_t offset, uint16_t val1, uint16_t val2)
  {
    uint8_t buffer[4] = {(uint8_t)val1, (uint8_t)(val1 >> 8),
                         (uint8_t)val2, (uint8_t)(val2 >> 8)};
    segmentWrite(JrkG2Command::SetOverridableSettings, offset, 4, buffer);
  }

  // set multiplier and exponent together in one segment write
  // (slightly faster than separate calls to getOvrSetting16() and getOvrSetting8())
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
};
