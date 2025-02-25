// Copyright (C) 2024 Morgritech
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

/// @file move-by-angle-with-acceleration.ino
/// @brief Example showing how to move a stepper motor (with acceleration/deceleration) in oscillation by setting an angle, using the MT-arduino-stepper-driver library.

#include <stepper_driver.h>

// GPIO pins.
const uint8_t kPulPin = 11; ///< Output pin for the stepper driver PUL/STP/CLK (pulse/step) interface.
const uint8_t kDirPin = 12; ///< Output pin for the stepper driver DIR/CW (direction) interface.
const uint8_t kEnaPin = 13; ///< Output pin for the stepper driver ENA/EN (enable) interface.

// Serial properties.
const int kBaudRate = 9600; ///< The serial communication speed.

// Stepper motor/drive system properties.
const float kFullStepAngle_degrees = 1.8F; ///< The stepper motor full step angle in degrees. Obtained from the motor data sheet.
const float kGearRatio = 1.0F; ///< The system/stepper motor gear ratio (1 if not using a gearing system or a geared stepper motor).

// Stepper driver properties.
const uint16_t kMicrostepMode = 8; ///< Microstep mode (1/8). Remember to change the setting on the stepper driver to match.
// Minimum time (us) to delay after changing the state of a pin. Obtained from the stepper driver data sheet.
// These values are for the StepperOnline DM542T stepper driver, but should work for most stepper drivers.
const float kPulDelay_us = 2.5F; ///< Minimum delay (us) for the stepper driver PUL pin.
const float kDirDelay_us = 5.0F; ///< Minimum delay (us) for the stepper driver Dir pin.
const float kEnaDelay_us = 5.0F; ///< Minimum delay (us) for the stepper driver Ena pin.
// Sweep angle during oscillation.
const float kSweepAngle_degrees = 3600.0F; // 3600/360 = 10 revolutions sweep angle.
// Speed.
const float kSpeed_RPM = 150.0F; ///< Rotation speed (RPM).
// Acceleration/deceleration.
const float kAcceleration_microsteps_per_s_per_s = 7000.0; ///< Acceleration (microsteps per second-squared).

// Other properties.
const uint16_t kStartupTime_ms = 1000; ///< Minimum startup/boot time in milliseconds (ms); based on the stepper driver.

/// @brief Stepper Driver instance for the stepper motor.
mt::StepperDriver stepper_driver(kPulPin, kDirPin, kEnaPin, kMicrostepMode, kFullStepAngle_degrees, kGearRatio);
//mt::StepperDriver stepper_driver(kPulPin, kDirPin, kEnaPin); // Default values: microstep mode = 1, full step angle = 1.8, gear ratio = 1. 

/// @brief The main application entry point for initialisation tasks.
void setup() {
  // Initialise the Serial Port.
  Serial.begin(kBaudRate);

  // Initialise the output pins.
  pinMode(kPulPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  pinMode(kEnaPin, OUTPUT);

  // Set the expected behaviour of the stepper driver pins.
  // If these are not set, default values from the library will be used.
  stepper_driver.set_ena_pin_enabled_state(mt::StepperDriver::PinState::kLow); // The pin state for giving power to the motor.
  stepper_driver.set_dir_pin_positive_direction_state(mt::StepperDriver::PinState::kHigh); // The pin state for a "positive" motion direction.

  // Set the acceleration algorithm to be used.
  // If this is not set, the default value from the library will be used.
  stepper_driver.set_acceleration_algorithm(mt::StepperDriver::AccelerationAlgorithm::kMorgridge24);

  // Set stepper driver properties.
  // If these are not set, default values from the library will be used.
  stepper_driver.set_pul_delay_us(kPulDelay_us);
  stepper_driver.set_dir_delay_us(kDirDelay_us);
  stepper_driver.set_ena_delay_us(kEnaDelay_us);
  stepper_driver.SetSpeed(kSpeed_RPM, mt::StepperDriver::SpeedUnits::kRevolutionsPerMinute);
  stepper_driver.SetAcceleration(kAcceleration_microsteps_per_s_per_s,
                                  mt::StepperDriver::AccelerationUnits::kMicrostepsPerSecondPerSecond);

  // Activate the stepper driver.
  //stepper_driver.set_power_state(mt::StepperDriver::PowerState::kEnabled)// This is usually activated by default, hence this may not be required.

  // Delay for the startup time.
  delay(kStartupTime_ms);

  Serial.println(F("\n...Setup complete...\n"));
}

/// @brief The continuously running function for repetitive tasks.
void loop() {
  // Variable to keep track of the sweep angle and direction.
  static float sweep_angle_degrees = kSweepAngle_degrees;
  // Variable to keep track of the motion status.
  static mt::StepperDriver::MotionStatus motion_status;

  // Move the motor.
  motion_status = stepper_driver.MoveByAngle(sweep_angle_degrees, mt::StepperDriver::AngleUnits::kDegrees, mt::StepperDriver::MotionType::kRelative); // This must be called repeatedly.
  
  if (motion_status == mt::StepperDriver::MotionStatus::kIdle) {
    // Motion has completed.
    // Change direction.
    sweep_angle_degrees = -1.0 * sweep_angle_degrees;
    // OR Stop the motion.
    //motion_type = mt::StepperDriver::MotionType::kStopAndReset; // Uncomment this section to stop the motion instead.    
  }
}