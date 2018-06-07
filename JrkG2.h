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
  SoftOvercurrent     = 6,
  SerialSignal        = 7,
  SerialOverrun       = 8,
  SerialBufferFull    = 9,
  SerialCrc           = 10,
  SerialProtocol      = 11,
  SerialTimeout       = 12,
  HardOvercurrent     = 13,
};

/// This enum defines the Jrk G2 command bytes which are used for its serial and
/// I2C interfaces.  These bytes are used by the library and you should not need
/// to use them.
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
  GetEEPROMSettings                 = 0xE3,
  GetVariables                      = 0xE5,
  SetRAMSettings                    = 0xE6,
  GetRAMSettings                    = 0xEA,
  GetCurrentChoppingOccurrenceCount = 0xEC,
};

/// This enum defines the modes in which the Jrk G2's duty cycle target or duty
/// cycle, normally derived from the output of its PID algorithm, can be
/// overridden with a forced value.
///
/// See JrkG2Base::getForceMode(), JrkG2Base::forceDutyCycleTarget(), and
/// JrkG2Base::forceDutyCycle().
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
  /// Returns 0 if the last communication with the device was successful, and
  /// non-zero if there was an error.
  uint8_t getLastError()
  {
    return _lastError;
  }

  ///\name Motor control commands
  ///@{

  /// Sets the target of the Jrk to a value in the range 0 to 4095.
  ///
  /// The target can represent a target duty cycle, speed, or position depending
  /// on the feedback mode.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTarget(3000);
  /// ```
  ///
  /// This functions sends a "Set target" command to the jrk, which clears the
  /// "Awaiting command" error bit and (if the input mode is serial) will set
  /// the jrk's input and target variables.
  ///
  /// See also setTargetLowResRev(), setTargetLowResFwd(), getTarget(),
  /// forceDutyCycleTarget(), forceDutyCycle().
  void setTarget(uint16_t target)
  {
    // lower 5 bits in command byte
    // upper 7 bits in data byte
    if (target > 4095) { target = 4095; }
    commandW7((uint8_t)JrkG2Command::SetTarget | (target & 0x1F), target >> 5);
  }

  /// Sets the target of the Jrk based on a value in the range 0 to 127.
  ///
  /// If the value is zero, then this command is equivalent to the "Stop motor"
  /// command. Otherwise, the value maps to a 12-bit target less than 2048.
  ///
  /// If the feedback mode is Analog or Tachometer, then the formula is
  /// Target = 2048 - 16 * value.
  ///
  /// If the feedback mode is None (speed control mode), then the formula is
  /// Target = 2048 - (600 / 127) * value.  This means that a value of
  /// 127 will set the duty cycle target to full-speed reverse (-600).
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTargetLowResRev(100);
  /// ```
  ///
  /// This function sends a "Set target low resolution reverse" command to the
  /// Jrk G2, which clears the "Awaiting command" error bit and (if the input
  /// mode is serial) will set the jrk's input and target variables.
  ///
  /// See also setTargetLowResFwd(), setTarget(), getTarget(), and stopMotor().
  void setTargetLowResRev(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResRev, target);
  }

  /// Sets the target of the Jrk based on a value in the range 0 to 127 that
  /// maps to a 12-bit target of 2048 or greater.
  ///
  /// If the feedback mode is Analog or Tachometer, then the formula is
  /// Target = 2048 + 16 * value.
  ///
  /// If the feedback mode is None (speed control mode), then the formula is
  /// Target = 2048 + (600 / 127) * value.  This means that a value of 127 will
  /// set the duty cycle target to full-speed reverse (-600), while a value of
  /// zero will make the motor stop.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.setTargetLowResFwd(100);
  /// ```
  ///
  /// This function sends a "Set target low resolution forward" command to the
  /// Jrk G2, which clears the "Awaiting command" error bit and (if the input
  /// mode is serial) will set the jrk's input and target variables.
  ///
  /// See also setTargetLowResRev(), setTarget(), and getTarget().
  void setTargetLowResFwd(uint8_t target)
  {
    if (target > 127) { target = 127; }
    commandW7(JrkG2Command::SetTargetLowResFwd, target);
  }

  /// Forces the duty cycle target of the Jrk to a value in the range
  /// -600 to +600.
  ///
  /// The Jrk will ignore the results of the usual algorithm for choosing the duty
  /// cycle target, and instead set it to be equal to the target specified by this
  /// command.  The Jrk will set its 'Integral' variable to 0 while in this mode.
  ///
  /// This is useful if the Jrk is configured to use feedback but you want to take
  /// control of the motor for some time, while still respecting errors and motor
  /// limits as usual.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.forceDutyCycleTarget(250);
  /// ```
  ///
  /// This function sends a "Force duty cycle target" command to the Jrk G2, which
  /// clears the "Awaiting command" error bit.
  ///
  /// To get out of this mode, use setTarget(), setTargetLowResFwd(),
  /// setTargetLowResRev(), forceDutyCycle(), or stopMotor().
  ///
  /// See also getForceMode().
  void forceDutyCycleTarget(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycleTarget, dutyCycle);
  }

  /// Forces the duty cycle of the Jrk to a value in the range -600 to +600.
  ///
  /// The jrk will ignore the results of the usual algorithm for choosing the
  /// duty cycle, and instead set it to be equal to the value specified by this
  /// command, ignoring all motor limits except the maximum duty cycle
  /// parameters, and ignoring the 'Input invalid', 'Input disconnect', and
  /// 'Feedback disconnect' errors.  This command will have an immediate effect,
  /// regardless of the PID period.  The jrk will set its 'Integral' variable to
  /// 0 while in this mode.
  ///
  /// This is useful if the jrk is configured to use feedback but you want to take
  /// control of the motor for some time, without respecting most motor limits.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.forceDutyCycle(250);
  /// ```
  ///
  /// This function sends a "Force duty cycle" command to the Jrk G2, which
  /// clears the "Awaiting command" error bit.
  ///
  /// To get out of this mode, use setTarget(), setTargetLowResFwd(),
  /// setTargetLowResRev(), forceDutyCycleTarget(), or stopMotor().
  ///
  /// See also getForceMode().
  void forceDutyCycle(int16_t dutyCycle)
  {
    if (dutyCycle > 600) { dutyCycle = 600; }
    if (dutyCycle < -600) { dutyCycle = -600; }
    commandWs14(JrkG2Command::ForceDutyCycle, dutyCycle);
  }

  /// Turns the motor off.
  ///
  /// This function sends a "Stop motor" command to the Jrk, which sets the
  /// "Awaiting command" error bit.  The Jrk will respect the configured
  /// deceleration limit while decelerating to a duty cycle of 0, unless the
  /// "Awaiting command" error has been configured as a hard error.  Once the duty
  /// cycle reaches zero, the Jrk will either brake or coast.
  ///
  /// Example usage:
  /// ```
  /// jrkG2.stopMotor();
  /// ```
  void stopMotor()
  {
    commandQuick(JrkG2Command::MotorOff);
  }

  ///@}

  ///\name Variable reading commands
  ///@{

  /// Gets the input variable.
  ///
  /// The input variable is a raw, unscaled value representing a measurement
  /// taken by the Jrk of the input to the system.  In serial input mode, the
  /// input is equal to the target, which can be set to any value from 0 to 4095
  /// using serial commands.  In analog input mode, the input is a measurement
  /// of the voltage on the SDA pin, where 0 is 0 V and 4092 is a voltage equal
  /// to the Jrk's 5V pin (approximately 4.8 V).  In RC input mode, the input is
  /// the duration of the last RC pulse measured, in units of 2/3 us.
  ///
  /// See the Jrk G2 user's guide for more information about input modes.
  ///
  /// See also getTarget() and setTarget().
  uint16_t getInput()
  {
    return getVar16SingleByte(VarOffset::Input);
  }

  /// Gets the target variable.
  ///
  /// In serial input mode, the target is set directly with serial commands.  In
  /// the other input modes, the target is computed by scaling the input, using
  /// the configurable input scaling settings.
  ///
  /// See also setTarget() and getInput().
  uint16_t getTarget()
  {
    return getVar16SingleByte(VarOffset::Target);
  }

  /// Gets the feedback variable.
  ///
  /// The feedback variable is a raw, unscaled feedback value, representing a
  /// measurement taken by the Jrk of the output of the system.  In analog
  /// feedback mode, the feedback is a measurement of the voltage on the FBA
  /// pin, where 0 is 0 V and 4092 is a voltage equal to the Jrk's 5V pin
  /// (approximately 4.8 V).  In frequency feedback mode, the feedback is 2048
  /// plus or minus a measurement of the frequency of pulses on the FBT pin.  In
  /// feedback mode none (open-loop speed control mode), the feedback is always
  /// zero.
  ///
  /// See also getScaledFeedback().
  uint16_t getFeedback()
  {
    return getVar16SingleByte(VarOffset::Feedback);
  }

  /// Gets the scaled feedback variable.
  ///
  /// The scaled feedback is calculated from the feedback using the Jrk's
  /// configurable feedback scaling settings.
  ///
  /// See also getFeedback().
  uint16_t getScaledFeedback()
  {
    return getVar16SingleByte(VarOffset::ScaledFeedback);
  }

  /// Gets the integral variable.
  ///
  /// In general, every PID period, the error (scaled feedback minus target) is
  /// added to the integral (also known as error sum).  There are several
  /// settings to configure the behavior of this variable, and it is used in the
  /// PID calculation.
  int16_t getIntegral()
  {
    return getVar16SingleByte(VarOffset::Integral);
  }

  /// Gets the duty cycle target variable.
  ///
  /// In general, this is the duty cycle that the Jrk is trying to achieve.  A
  /// value of -600 or less means full speed reverse, while a value of 600 or
  /// more means full speed forward.  A value of 0 means stopped (braking or
  /// coasting).  In no feedback mode (open-loop speed control mode), the duty
  /// cycle target is normally the target minus 2048. In other feedback modes,
  /// the duty cycle target is normally the sum of the proportional, integral,
  /// and derivative terms of the PID algorithm.  In any mode, the duty cycle
  /// target can be overridden with forceDutyCycleTarget().
  ///
  /// If an error is stopping the motor, the duty cycle target variable will not
  /// be directly affected, but the duty cycle variable will change/decelerate
  /// to zero.
  ///
  /// See also getDutyCycle(), getLastDutyCycle(), and forceDutyCycleTarget().
  int16_t getDutyCycleTarget()
  {
    return getVar16SingleByte(VarOffset::DutyCycleTarget);
  }

  /// Gets the duty cycle variable.
  ///
  /// The duty cycle variable is the duty cycle at which the jrk is currently
  /// driving the motor.  A value of -600 means full speed reverse, while a
  /// value of 600 means full speed forward.  A value of 0 means stopped
  /// (braking or coasting).  The duty cycle could be different from the duty
  /// cycle target because it normally takes into account the Jrk's configurable
  /// motor limits and errors.  The duty cycle can be overridden with
  /// forceDutyCycle().
  ///
  /// See also getLastDutyCycle(), getDutyCycleTarget(), and forceDutyCycle().
  int16_t getDutyCycle()
  {
    return getVar16SingleByte(VarOffset::DutyCycle);
  }

  /// Gets the most-significant 8 bits of the "Current" variable.
  ///
  /// The Jrk G2 supports this command mainly to be compatible with older Jrk
  /// models.  In new applications, we recommend using getCurrent(), which
  /// provides a higher-resolution measurement.
  uint8_t getCurrentLowRes()
  {
    return getVar8SingleByte(VarOffset::CurrentLowRes);
  }

  /// Returns true if the Jrk's most recent PID cycle took more time than the
  /// configured PID period.  This indicates that the Jrk does not have time to
  /// perform all of its tasks at the desired rate.  Most often, this is caused
  /// by the configured number of analog samples for input, feedback, or current
  /// sensing being too high for the configured PID period.
  bool getPIDPeriodExceeded()
  {
    return getVar8SingleByte(VarOffset::PIDPeriodExceeded);
  }

  /// Get the "PID period count" variable, which is the number of PID periods
  /// that have elapsed.  It resets to 0 after reaching 65535.  The duration of
  /// the PID period can be configured.
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
  /// are defined in the ::JrkG2Error enum.
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
  /// It is possible to read this variable without clearing the bits in it using
  /// a getVariables().
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
  /// It is possible to read this variable without clearing the bits in it using
  /// getVariables().
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
  ///
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
  /// milliamps.
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

  /// Gets the raw RC pulse width measured on the Jrk's RC input, in units of
  /// twelfths of a microsecond.
  ///
  /// Returns 0 if the RC input is missing or invalid.
  ///
  /// Example usage:
  /// ```
  /// uint16_t pulseWidth = jrk.getRCPulseWidth();
  /// if (pulseWidth != 0 && pulseWidth < 18000)
  /// {
  ///   // Input is valid and pulse width is less than 1500 microseconds.
  /// }
  /// ```
  uint16_t getRCPulseWidth()
  {
    return getVar16(VarOffset::RCPulseWidth);
  }

  /// Gets the raw pulse rate or pulse width measured on the Jrk's FBT
  /// (tachometer feedback) pin.
  ///
  /// In pulse counting mode, this will be the number of pulses on the FBT pin
  /// seen in the last N PID periods, where N is the "Pulse samples" setting.
  ///
  /// In pulse timing mode, this will be a measurement of the width of pulses on
  /// the FBT pin.  This measurement is affected by several configurable
  /// settings.
  ///
  /// Example usage:
  /// ```
  /// uint16_t fbtReading = jrk.getFBTReading();
  /// ```
  uint16_t getFBTReading()
  {
    return getVar16(VarOffset::FBTReading);
  }

  /// Gets the analog reading from the specified pin.
  ///
  /// The reading is left-justified, so 0xFFFE represents a voltage equal to the
  /// Jrk's 5V pin (approximately 4.8 V).
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
  /// A return value of 0 means low while 1 means high.  In most cases, pins
  /// configured as analog inputs cannot be read as digital inputs, so their
  /// values will be 0.  See getAnalogReading() for those pins.
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

  /// Gets the Jrk's raw measurement of the current running through the motor.
  ///
  /// This is an analog voltage reading from the Jrk's current sense
  /// pin.  The units of the reading depend on what hard current limit is being
  /// used (getEncodedHardCurrentLimit()).
  ///
  /// See also getCurrent().
  uint16_t getRawCurrent()
  {
    return getVar16(VarOffset::RawCurrent);
  }

  /// Gets the encoded value representing the hardware current limit the jrk is
  /// currently using.
  ///
  /// See also setEncodedHardCurrentLimit(), getCurrent().
  uint16_t getEncodedHardCurrentLimit()
  {
    return getVar16(VarOffset::EncodedHardCurrentLimit);
  }

  /// Gets the duty cycle the Jrk drove the motor with in the last PID period.
  ///
  /// This can be useful for converting the getRawCurrent() reading into
  /// milliamps.
  ///
  /// See also getDutyCycle(), getDutyCycleTarget(), and getCurrent().
  int16_t getLastDutyCycle()
  {
    return getVar16(VarOffset::LastDutyCycle);
  }

  /// Gets the number of consecutive PID periods during which current chopping
  /// due to the hard current limit has been active.
  ///
  /// See also getCurrentChoppingOccurrenceCount().
  uint8_t getCurrentChoppingConsecutiveCount()
  {
    return getVar8(VarOffset::CurrentChoppingConsecutiveCount);
  }

  /// Gets and clears the "Current chopping occurrence count" variable, which is
  /// the number of PID periods during which current chopping due to the hard
  /// current limit has been active, since the last time the variable was
  /// cleared.
  ///
  /// See also getCurrentChoppingConsecutiveCount().
  uint8_t getCurrentChoppingOccurrenceCount()
  {
    return commandR8(JrkG2Command::GetCurrentChoppingOccurrenceCount);
  }
  ///@}

  ///\name RAM settings commands
  ///@{

  /// Sets or clears the "Reset integral" setting in the Jrk's RAM settings.
  ///
  /// If this setting is set to true, the PID algorithm will reset the integral
  /// variable (also known as error sum) when the absolute value of the
  /// proportional term exceeds 600.
  ///
  /// When enabled, this can help limit integral wind-up, or the uncontrolled
  /// growth of the integral when the feedback system is temporarily unable to
  /// keep the error small. This might happen, for example, when the target is
  /// changing quickly.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getResetIntegral().
  void setResetIntegral(bool reset)
  {
    uint8_t tmp = getRAMSetting8(SettingOffset::OptionsByte3);
    if (getLastError()) { return; }
    if (reset)
    {
      tmp |= 1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral;
    }
    else
    {
      tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::ResetIntegral);
    }
    setRAMSetting8(SettingOffset::OptionsByte3, tmp);
  }

  /// Gets the "Reset integral" setting from the Jrk's RAM settings.
  ///
  /// See also setResetIntegral().
  bool getResetIntegral()
  {
    return getRAMSetting8(SettingOffset::OptionsByte3) >>
      (uint8_t)JrkG2OptionsByte3::ResetIntegral & 1;
  }

  /// Sets or clears the "Coast when off" setting in the Jrk's RAM settings.
  ///
  /// By default, the Jrk drives both motor outputs low when the motor is
  /// stopped (duty cycle is zero), causing it to brake.  If enabled, this
  /// setting causes it to instead tri-state both outputs, making the motor
  /// coast.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getCoastWhenOff().
  void setCoastWhenOff(bool coast)
  {
    uint8_t tmp = getRAMSetting8(SettingOffset::OptionsByte3);
    if (getLastError()) { return; }
    if (coast)
    {
      tmp |= 1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff;
    }
    else
    {
      tmp &= ~(1 << (uint8_t)JrkG2OptionsByte3::CoastWhenOff);
    }
    setRAMSetting8(SettingOffset::OptionsByte3, tmp);
  }

  /// Gets the "Coast when off" setting from the Jrk's RAM settings.
  ///
  /// See also setCoastWhenOff().
  bool getCoastWhenOff()
  {
    return getRAMSetting8(SettingOffset::OptionsByte3) >>
      (uint8_t)JrkG2OptionsByte3::CoastWhenOff & 1;
  }

  /// Sets the proportional coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// Example usage:
  /// ```
  /// // Set the proportional coefficient to 1.125 (9/(2^3)).
  /// jrk.setProportionalCoefficient(9, 3);
  /// ```
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getProportionalMultiplier() and getProportionalExponent(), as
  /// well as setIntegralCoefficient() and setDerivativeCoefficient().
  void setProportionalCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(SettingOffset::ProportionalMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the proportional coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getProportionalExponent() and setProportionalCoefficient().
  uint16_t getProportionalMultiplier()
  {
    return getRAMSetting16(SettingOffset::ProportionalMultiplier);
  }

  /// Gets the exponent part of the proportional coefficient from the Jrk's RAM
  /// settings.
  ///
  /// See also getProportionalMultiplier() and setProportionalCoefficient().
  uint8_t getProportionalExponent()
  {
    return getRAMSetting8(SettingOffset::ProportionalExponent);
  }

  /// Sets the integral coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getIntegralMultiplier() and getIntegralExponent(), as
  /// well as setProportionalCoefficient() and setDerivativeCoefficient().
  void setIntegralCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(SettingOffset::IntegralMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the integral coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getIntegralExponent() and setIntegralCoefficient().
  uint16_t getIntegralMultiplier()
  {
    return getRAMSetting16(SettingOffset::IntegralMultiplier);
  }

  /// Gets the exponent part of the integral coefficient from the Jrk's
  /// RAM settings.
  ///
  /// See also getIntegralMultiplier() and setIntegralCoefficient().
  uint8_t getIntegralExponent()
  {
    return getRAMSetting8(SettingOffset::IntegralExponent);
  }

  /// Sets the derivative coefficient in the Jrk's RAM settings.
  ///
  /// This coefficient is used in the Jrk's PID algorithm.  The coefficient
  /// takes the form:
  ///
  /// multiplier / (2 ^ exponent)
  ///
  /// The multiplier can range from 0 to 1023, and the exponent can range
  /// from 0 to 18.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getDerivativeMultiplier() and getDerivativeExponent(), as
  /// well as setProportionalCoefficient() and setIntegralCoefficient().
  void setDerivativeCoefficient(uint16_t multiplier, uint8_t exponent)
  {
    setPIDCoefficient(SettingOffset::DerivativeMultiplier, multiplier, exponent);
  }

  /// Gets the multiplier part of the derivative coefficient from the
  /// Jrk's RAM settings.
  ///
  /// See also getDerivativeExponent() and setDerivativeCoefficient().
  uint16_t getDerivativeMultiplier()
  {
    return getRAMSetting16(SettingOffset::DerivativeMultiplier);
  }

  /// Gets the exponent part of the derivative coefficient from the
  /// Jrk's RAM settings.
  ///
  /// See also getDerivativeMultiplier() and setDerivativeCoefficient().
  uint8_t getDerivativeExponent()
  {
    return getRAMSetting8(SettingOffset::DerivativeExponent);
  }

  /// Sets the PID period in the Jrk's RAM settings.
  ///
  /// This is the rate at which the Jrk runs through all of its calculations, in
  /// milliseconds.  Note that a higher PID period will result in a more slowly
  /// changing integral and a higher derivative, so the two corresponding PID
  /// coefficients might need to be adjusted whenever the PID period is changed.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getPIDPeriod().
  void setPIDPeriod(uint16_t period)
  {
    setRAMSetting16(SettingOffset::PIDPeriod, period);
  }

  /// Gets the PID period from the Jrk's RAM settings, in milliseconds.
  ///
  /// See also setPIDPeriod().
  uint16_t getPIDPeriod()
  {
    return getRAMSetting16(SettingOffset::PIDPeriod);
  }

  /// Sets the integral limit in the Jrk's RAM settings.
  ///
  /// The PID algorithm prevents the absolute value of the integral variable
  /// (also known as error sum) from exceeding this limit.  This can help limit
  /// integral wind-up.  The limit can range from 0 to 32767.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getIntegralLimit().
  void setIntegralLimit(uint16_t limit)
  {
    setRAMSetting16(SettingOffset::IntegralLimit, limit);
  }

  /// Gets the integral limit from the Jrk's RAM settings.
  ///
  /// See also setIntegralLimit().
  uint16_t getIntegralLimit()
  {
    return getRAMSetting16(SettingOffset::IntegralLimit);
  }

  /// Sets the maximum duty cycle while feedback is out of range in the Jrk's
  /// RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleWhileFeedbackOutOfRange().
  void setMaxDutyCycleWhileFeedbackOutOfRange(uint16_t duty)
  {
    setRAMSetting16(SettingOffset::MaxDutyCycleWhileFeedbackOutOfRange, duty);
  }

  /// Gets the maximum duty cycle while feedback is out of range from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDutyCycleWhileFeedbackOutOfRange().
  uint16_t getMaxDutyCycleWhileFeedbackOutOfRange()
  {
    return getRAMSetting16(SettingOffset::MaxDutyCycleWhileFeedbackOutOfRange);
  }

  /// Sets the maximum acceleration in the forward direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxAccelerationForward(), setMaxAccelerationReverse(),
  /// setMaxAcceleration(), and setMaxDecelerationForward().
  void setMaxAccelerationForward(uint16_t accel)
  {
    setRAMSetting16(SettingOffset::MaxAccelerationForward, accel);
  }

  /// Gets the maximum acceleration in the forward direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxAccelerationForward().
  uint16_t getMaxAccelerationForward()
  {
    return getRAMSetting16(SettingOffset::MaxAccelerationForward);
  }

  /// Sets the maximum acceleration in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxAccelerationReverse(), setMaxAccelerationForward(),
  /// setMaxAcceleration(), and setMaxDecelerationReverse().
  void setMaxAccelerationReverse(uint16_t accel)
  {
    setRAMSetting16(SettingOffset::MaxAccelerationReverse, accel);
  }

  /// Gets the maximum acceleration in the reverse direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxAccelerationReverse().
  uint16_t getMaxAccelerationReverse()
  {
    return getRAMSetting16(SettingOffset::MaxAccelerationReverse);
  }

  /// Sets the maximum acceleration in both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxAccelerationForward(), setMaxAccelerationReverse(),
  /// setMaxDeceleration().
  void setMaxAcceleration(uint16_t accel)
  {
    setRAMSetting16x2(SettingOffset::MaxAccelerationForward, accel, accel);
  }

  /// Sets the maximum deceleration in the forward direction in the Jrk's RAM
  /// settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDecelerationForward(), setMaxDecelerationReverse(),
  /// setMaxDeceleration(), and setMaxAccelerationForward().
  void setMaxDecelerationForward(uint16_t decel)
  {
    setRAMSetting16(SettingOffset::MaxDecelerationForward, decel);
  }

  /// Gets the maximum deceleration in the forward direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDecelerationForward().
  uint16_t getMaxDecelerationForward()
  {
    return getRAMSetting16(SettingOffset::MaxDecelerationForward);
  }

  /// Sets the maximum deceleration in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDecelerationReverse(), setMaxDecelerationForward(),
  /// setMaxDeceleration(), and setMaxAccelerationReverse().
  void setMaxDecelerationReverse(uint16_t decel)
  {
    setRAMSetting16(SettingOffset::MaxDecelerationReverse, decel);
  }

  /// Gets the maximum deceleration in the reverse direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDecelerationReverse().
  uint16_t getMaxDecelerationReverse()
  {
    return getRAMSetting16(SettingOffset::MaxDecelerationReverse);
  }

  /// Sets the maximum deceleration in both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxDecelerationForward(), setMaxDecelerationReverse(),
  /// setMaxAcceleration().
  void setMaxDeceleration(uint16_t decel)
  {
    setRAMSetting16x2(SettingOffset::MaxDecelerationForward, decel, decel);
  }

  /// Sets the maximum duty cycle in the forward direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleForward(), setMaxDutyCycleReverse().
  void setMaxDutyCycleForward(uint16_t duty)
  {
    setRAMSetting16(SettingOffset::MaxDutyCycleForward, duty);
  }

  /// Gets the maximum duty cycle in the forward direction from the Jrk's RAM
  /// settings.
  ///
  /// See also setMaxDutyCycleForward().
  uint16_t getMaxDutyCycleForward()
  {
    return getRAMSetting16(SettingOffset::MaxDutyCycleForward);
  }

  /// Sets the maximum duty cycle in the reverse direction in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getMaxDutyCycleReverse(), setMaxDutyCycleForard().
  void setMaxDutyCycleReverse(uint16_t duty)
  {
    setRAMSetting16(SettingOffset::MaxDutyCycleReverse, duty);
  }

  /// Gets the maximum duty cycle in the reverse direction from the
  /// Jrk's RAM settings.
  ///
  /// See also setMaxDutyCycleReverse().
  uint16_t getMaxDutyCycleReverse()
  {
    return getRAMSetting16(SettingOffset::MaxDutyCycleReverse);
  }

  /// Sets the maximum duty cycle for both directions in the
  /// Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setMaxDutyCycleForward(), setMaxDutyCycleReverse().
  void setMaxDutyCycle(uint16_t duty)
  {
    setRAMSetting16x2(SettingOffset::MaxDutyCycleForward, duty, duty);
  }

  /// Sets the encoded hard current limit for driving in the forward direction
  /// in the Jrk's RAM settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getEncodedHardCurrentLimitForward() and
  /// setEncodedHardCurrentLimitReverse().
  void setEncodedHardCurrentLimitForward(uint16_t encoded_limit)
  {
    setRAMSetting16(SettingOffset::EncodedHardCurrentLimitForward,
      encoded_limit);
  }

  /// Gets the encoded hard current limit for driving in the forward direction
  /// from the Jrk's RAM settings.
  ///
  /// See also setEncodedHardCurrentLimitForward().
  uint16_t getEncodedHardCurrentLimitForward()
  {
    return getRAMSetting16(SettingOffset::EncodedHardCurrentLimitForward);
  }

  /// Sets the encoded hard current limit for driving in the reverse direction
  /// in the Jrk's RAM settings
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getEncodedHardCurrentLimitReverse() and
  /// setEncodedHardCurrentLimitForward().
  void setEncodedHardCurrentLimitReverse(uint16_t encoded_limit)
  {
    setRAMSetting16(SettingOffset::EncodedHardCurrentLimitReverse, encoded_limit);
  }

  /// Gets the encoded hard current limit for driving in the reverse direction
  /// from the Jrk's RAM settings.
  ///
  /// See also setEncodedHardCurrentLimitReverse().
  uint16_t getEncodedHardCurrentLimitReverse()
  {
    return getRAMSetting16(SettingOffset::EncodedHardCurrentLimitReverse);
  }

  /// Sets the encoded hard current limit for both directions in the Jrk's RAM
  /// settings.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setEncodedHardCurrentLimitForward(),
  /// setEncodedHardCurrentLimitReverse(), getEncodedHardCurrentLimit(), and
  /// setSoftCurrentLimit().
  void setEncodedHardCurrentLimit(uint16_t encoded_limit)
  {
    setRAMSetting16x2(SettingOffset::EncodedHardCurrentLimitForward,
      encoded_limit, encoded_limit);
  }

  /// Sets the brake duration when switching from forward to reverse in the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getBrakeDurationForward() and setBrakeDurationReverse().
  void setBrakeDurationForward(uint8_t duration)
  {
    setRAMSetting8(SettingOffset::BrakeDurationForward, duration);
  }

  /// Gets the brake duration when switching from forward to reverse from the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// See also setBrakeDurationForward().
  uint8_t getBrakeDurationForward()
  {
    return getRAMSetting8(SettingOffset::BrakeDurationForward);
  }

  /// Sets the brake duration when switching from reverse to forward in the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getBrakeDurationReverse() and setBrakeDurationForward().
  void setBrakeDurationReverse(uint8_t duration)
  {
    setRAMSetting8(SettingOffset::BrakeDurationReverse, duration);
  }

  /// Gets the brake duration when switching from reverse to forward from the
  /// Jrk's RAM settings, in units of 5 ms.
  ///
  /// See also setBrakeDurationReverse().
  uint8_t getBrakeDurationReverse()
  {
    return getRAMSetting8(SettingOffset::BrakeDurationReverse);
  }

  /// Sets the brake duration for both directions in the Jrk's RAM settings, in
  /// units of 5 ms.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setBrakeDurationForward(), setBrakeDurationReverse().
  void setBrakeDuration(uint8_t duration)
  {
    setRAMSetting8x2(SettingOffset::BrakeDurationForward, duration, duration);
  }

  /// Sets the soft current limit when driving in the forward direction in the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getSoftCurrentLimitForward() and setSoftCurrentLimitReverse().
  void setSoftCurrentLimitForward(uint16_t current)
  {
    setRAMSetting16(SettingOffset::SoftCurrentLimitForward, current);
  }

  /// Gets the soft current limit when driving in the forward direction from the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// See also setSoftCurrentLimitForward().
  uint16_t getSoftCurrentLimitForward()
  {
    return getRAMSetting16(SettingOffset::SoftCurrentLimitForward);
  }

  /// Sets the soft current limit when driving in the reverse direction in the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also getSoftCurrentLimitReverse() and setSoftCurrentLimitForward().
  void setSoftCurrentLimitReverse(uint16_t current)
  {
    setRAMSetting16(SettingOffset::SoftCurrentLimitReverse, current);
  }

  /// Gets the soft current limit when driving in the reverse direction from the
  /// Jrk's RAM settings, in units of mA.
  ///
  /// See also setSoftCurrentLimitReverse().
  uint16_t getSoftCurrentLimitReverse()
  {
    return getRAMSetting16(SettingOffset::SoftCurrentLimitReverse);
  }

  /// Sets the soft current limit for driving in both directions in the Jrk's
  /// RAM settings, in units of mA.
  ///
  /// You would normally configure this setting ahead of time using the Jrk G2
  /// Configuration Utility, but this function allows you to change it
  /// temporarily on the fly.
  ///
  /// See also setSoftCurrentLimitForward() and setSoftCurrentLimitReverse(),
  /// setEncodedHardCurrentLimit().
  void setSoftCurrentLimit(uint16_t current)
  {
    setRAMSetting16x2(SettingOffset::SoftCurrentLimitForward, current, current);
  }

  ///@}

  ///\name Low-level settings and variables commands
  ///@{

  /// Gets a contiguous block of settings from the Jrk G2's EEPROM.
  ///
  /// The maximum length that can be fetched is 15 bytes.
  ///
  /// Example usage:
  /// ```
  /// // Get the Jrk's serial device number.
  /// uint8_t deviceNumber;
  /// jrk.getEEPROMSettings(0x28, 1, &deviceNumber);
  /// ```
  ///
  /// For information on how the settings are encoded,
  /// see the Jrk G2 user's guide.
  void getEEPROMSettings(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentRead(JrkG2Command::GetEEPROMSettings, offset, length, buffer);
  }

  /// Gets a contiguous block of settings from the Jrk G2's RAM.
  ///
  /// The maximum length that can be fetched is 15 bytes.
  ///
  /// Example usage:
  /// ```
  /// // Get the Jrk's feedback maximum setting.
  /// uint8_t buffer[2];
  /// jrk.getRAMSettings(0x1F, 2, buffer);
  /// uint16_t feedbackMaximum = buffer[0] + (buffer[1] << 8);
  /// ```
  ///
  /// Note that this library has several functions for reading and writing
  /// specific RAM settings, and they are easier to use than this function.
  ///
  /// For information on how the settings are encoded,
  /// see the Jrk G2 user's guide.
  void getRAMSettings(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentRead(JrkG2Command::GetRAMSettings, offset, length, buffer);
  }

  /// Sets a contiguous block of settings in the Jrk G2's RAM.
  ///
  /// The maximum length that can be written in a single command
  /// is 7 bytes over Serial, 13 bytes over I2C.
  ///
  /// Example usage:
  /// ```
  /// // Set the Jrk's feedback maximum setting.
  /// uint16_t feedbackMaximum = 1234;
  /// uint8_t buffer[2];
  /// buffer[0] = feedbackMaximum & 0xFF;
  /// buffer[1] = feedbackMaximum >> 8 & 0xFF;
  /// jrk.setRAMSettings(0x1F, 2, buffer);
  /// ```
  ///
  /// Note that this library has several functions for reading and writing
  /// specific RAM settings, and they are easier to use than this function.
  ///
  /// For information on how the settings are encoded,
  /// see the Jrk G2 user's guide.
  void setRAMSettings(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentWrite(JrkG2Command::SetRAMSettings, offset, length, buffer);
  }

  /// Gets a contiguous block of variables from the Jrk G2.
  ///
  /// Note that this library has convenient functions for reading every variable
  /// provided by the Jrk.  The main reason to use this function is if you want
  /// to read multiple variables at once for extra efficiency or to ensure that
  /// the variables are in a consistent state.
  ///
  /// The maximum length that can be fetched is 15 bytes.
  ///
  /// Example usage:
  /// ```
  /// // Get the Jrk's last device reset and its up time.
  /// uint8_t buffer[5];
  /// jrk.getVariables(0x1F, 5, buffer);
  /// ```
  ///
  /// For information on how the variables are encoded,
  /// see the Jrk G2 user's guide.
  void getVariables(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    segmentRead(JrkG2Command::GetVariables, offset, length, buffer);
  }

  ///@}

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
    FBTReading                      = 0x26, // uint16_t
    AnalogReadingSDA                = 0x28, // uint16_t
    AnalogReadingFBA                = 0x2A, // uint16_t
    DigitalReadings                 = 0x2C, // uint8_t
    RawCurrent                      = 0x2D, // uint16_t
    EncodedHardCurrentLimit         = 0x2F, // uint16_t
    LastDutyCycle                   = 0x31, // int16_t
    CurrentChoppingConsecutiveCount = 0x33, // uint8_t
    CurrentChoppingOccurrenceCount  = 0x34, // uint8_t; read with dedicated command
  };

  enum SettingOffset
  {
    OptionsByte1                        = 0x01,  // uint8_t
    OptionsByte2                        = 0x02,  // uint8_t
    InputMode                           = 0x03,  // uint8_t
    InputErrorMinimum                   = 0x04,  // uint16_t,
    InputErrorMaximum                   = 0x06,  // uint16_t,
    InputMinimum                        = 0x08,  // uint16_t,
    InputMaximum                        = 0x0A,  // uint16_t,
    InputNeutralMinimum                 = 0x0C,  // uint16_t,
    InputNeutralMaximum                 = 0x0E,  // uint16_t,
    OutputMinimum                       = 0x10,  // uint16_t,
    OutputNeutral                       = 0x12,  // uint16_t,
    OutputMaximum                       = 0x14,  // uint16_t,
    InputScalingDegree                  = 0x16,  // uint8_t,
    InputAnalogSamplesExponent          = 0x17,  // uint8_t,
    FeedbackMode                        = 0x18,  // uint8_t,
    FeedbackErrorMinimum                = 0x19,  // uint16_t,
    FeedbackErrorMaximum                = 0x1B,  // uint16_t,
    FeedbackMinimum                     = 0x1D,  // uint16_t,
    FeedbackMaximum                     = 0x1F,  // uint16_t,
    FeedbackDeadZone                    = 0x21,  // uint8_t,
    FeedbackAnalogSamplesExponent       = 0x22,  // uint8_t,
    SerialMode                          = 0x23,  // uint8_t,
    SerialBaudRateGenerator             = 0x24,  // uint16_t,
    SerialTimeout                       = 0x26,  // uint16_t,
    SerialDeviceNumber                  = 0x28,  // uint16_t,
    ErrorEnable                         = 0x2A,  // uint16_t
    ErrorLatch                          = 0x2C,  // uint16_t
    ErrorHard                           = 0x2E,  // uint16_t
    VinCalibration                      = 0x30,  // uint16_t
    PwmFrequency                        = 0x32,  // uint8_t
    CurrentSamplesExponent              = 0x33,  // uint8_t
    HardOvercurrentThreshold            = 0x34,  // uint8_t
    CurrentOffsetCalibration            = 0x35,  // uint16_t
    CurrentScaleCalibration             = 0x37,  // uint16_t
    FBTMethod                           = 0x39,  // uint8_t
    FBTOptions                          = 0x3A,  // uint8_t
    FBTTimingTimeout                    = 0x3B,  // uint16_t
    FBTSamples                          = 0x3D,  // uint8_t
    FBTDividerExponent                  = 0x3E,  // uint8_t
    IntegralDividerExponent             = 0x3F,  // uint8_t
    OptionsByte3                        = 0x50,  // uint8_t
    ProportionalMultiplier              = 0x51,  // uint16_t
    ProportionalExponent                = 0x53,  // uint8_t
    IntegralMultiplier                  = 0x54,  // uint16_t
    IntegralExponent                    = 0x56,  // uint8_t
    DerivativeMultiplier                = 0x57,  // uint16_t
    DerivativeExponent                  = 0x59,  // uint8_t
    PIDPeriod                           = 0x5A,  // uint16_t
    IntegralLimit                       = 0x5C,  // uint16_t
    MaxDutyCycleWhileFeedbackOutOfRange = 0x5E,  // uint16_t
    MaxAccelerationForward              = 0x60,  // uint16_t
    MaxAccelerationReverse              = 0x62,  // uint16_t
    MaxDecelerationForward              = 0x64,  // uint16_t
    MaxDecelerationReverse              = 0x66,  // uint16_t
    MaxDutyCycleForward                 = 0x68,  // uint16_t
    MaxDutyCycleReverse                 = 0x6A,  // uint16_t
    EncodedHardCurrentLimitForward      = 0x6C,  // uint16_t
    EncodedHardCurrentLimitReverse      = 0x6E,  // uint16_t
    BrakeDurationForward                = 0x70,  // uint8_t
    BrakeDurationReverse                = 0x71,  // uint8_t
    SoftCurrentLimitForward             = 0x72,  // uint16_t
    SoftCurrentLimitReverse             = 0x74,  // uint16_t
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

  void setRAMSetting8(uint8_t offset, uint8_t val)
  {
    segmentWrite(JrkG2Command::SetRAMSettings, offset, 1, &val);
  }

  void setRAMSetting16(uint8_t offset, uint16_t val)
  {
    uint8_t buffer[2] = {(uint8_t)val, (uint8_t)(val >> 8)};
    segmentWrite(JrkG2Command::SetRAMSettings, offset, 2, buffer);
  }

  void setRAMSetting8x2(uint8_t offset, uint8_t val1, uint8_t val2)
  {
    uint8_t buffer[2] = {val1, val2};
    segmentWrite(JrkG2Command::SetRAMSettings, offset, 2, buffer);
  }

  void setRAMSetting16x2(uint8_t offset, uint16_t val1, uint16_t val2)
  {
    uint8_t buffer[4] = {(uint8_t)val1, (uint8_t)(val1 >> 8),
                         (uint8_t)val2, (uint8_t)(val2 >> 8)};
    segmentWrite(JrkG2Command::SetRAMSettings, offset, 4, buffer);
  }

  // set multiplier and exponent together in one segment write
  // (slightly faster than separate calls to getRAMSetting16() and getRAMSetting8())
  void setPIDCoefficient(uint8_t offset, uint16_t multiplier, uint8_t exponent)
  {
    uint8_t buffer[3] = {(uint8_t)multiplier, (uint8_t)(multiplier >> 8), exponent};
    segmentWrite(JrkG2Command::SetRAMSettings, offset, 3, buffer);
  }

  uint8_t getRAMSetting8(uint8_t offset)
  {
    uint8_t result;
    segmentRead(JrkG2Command::GetRAMSettings, offset, 1, &result);
    return result;
  }

  uint16_t getRAMSetting16(uint8_t offset)
  {
    uint8_t buffer[2];
    segmentRead(JrkG2Command::GetRAMSettings, offset, 2, buffer);
    return ((uint16_t)buffer[0] << 0) | ((uint16_t)buffer[1] << 8);
  }

  // Convenience functions that take care of casting a JrkG2Command to a uint8_t.

  void commandQuick(JrkG2Command cmd)
  {
    commandQuick((uint8_t)cmd);
  }

  void commandW7(JrkG2Command cmd, uint8_t val)
  {
    commandW7((uint8_t)cmd, val);
  }

  void commandWs14(JrkG2Command cmd, int16_t val)
  {
    commandWs14((uint8_t)cmd, val);
  }

  uint8_t commandR8(JrkG2Command cmd)
  {
    return commandR8((uint8_t)cmd);
  }

  uint16_t commandR16(JrkG2Command cmd)
  {
    return commandR16((uint8_t)cmd);
  }

  void segmentRead(JrkG2Command cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer)
  {
    segmentRead((uint8_t)cmd, offset, length, buffer);
  }

  void segmentWrite(JrkG2Command cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer)
  {
    segmentWrite((uint8_t)cmd, offset, length, buffer);
  }

  // Low-level functions implemented by the serial/I2C subclasses.

  virtual void commandQuick(uint8_t cmd) = 0;
  virtual void commandW7(uint8_t cmd, uint8_t val) = 0;
  virtual void commandWs14(uint8_t cmd, int16_t val) = 0;
  virtual uint8_t commandR8(uint8_t cmd) = 0;
  virtual uint16_t commandR16(uint8_t cmd) = 0;
  virtual void segmentRead(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer) = 0;
  virtual void segmentWrite(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer) = 0;
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
  /// between 0 and 127, it specifies the device number to use in the Pololu
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
  /// JrkG2Serial jrk1(jrkG2Serial, 11);
  /// JrkG2Serial jrk2(jrkG2Serial, 12);
  /// ```
  JrkG2Serial(Stream & stream, uint8_t deviceNumber = 255) :
    _stream(&stream),
    _deviceNumber(deviceNumber)
  {
  }

  /// Gets the serial device number this object is using.
  uint8_t getDeviceNumber() { return _deviceNumber; }

private:
  Stream * const _stream;
  const uint8_t _deviceNumber;

  void commandQuick(uint8_t cmd) { sendCommandHeader(cmd); }
  void commandW7(uint8_t cmd, uint8_t val);
  void commandWs14(uint8_t cmd, int16_t val);
  uint8_t commandR8(uint8_t cmd);
  uint16_t commandR16(uint8_t cmd);
  void segmentRead(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer);
  void segmentWrite(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer);

  void sendCommandHeader(uint8_t cmd);
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
  ///
  /// This constructor only uses the least-significat 7 bits of the address,
  /// since I2C addresses can only go up to 127 and we want things to always
  /// just work as long as the address is the same as the "Device number".
  JrkG2I2C(uint8_t address = 11) : _address(address & 0x7F)
  {
  }

  // TODO: support Wire1 on Arduino Due, and bit-banging I2C on any board?

  /// Gets the I2C address this object is using.
  uint8_t getAddress() { return _address; }

private:
  const uint8_t _address;

  void commandQuick(uint8_t cmd);
  void commandW7(uint8_t cmd, uint8_t val);
  void commandWs14(uint8_t cmd, int16_t val);
  uint8_t commandR8(uint8_t cmd);
  uint16_t commandR16(uint8_t cmd);
  void segmentRead(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer);
  void segmentWrite(uint8_t cmd, uint8_t offset,
    uint8_t length, uint8_t * buffer) ;
};
