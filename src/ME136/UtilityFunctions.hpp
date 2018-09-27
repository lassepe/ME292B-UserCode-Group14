#pragma once

#include "MainLoopTypes.hpp"

enum class MotorID {
  // motorCommand1 -> located at body +x +y
  FRONT_LEFT = 1,
  // motorCommand2 -> located at body +x -y
  FRONT_RIGHT = 2,
  // motorCommand3 -> located at body -x -y
  REAR_RIGHT= 3,
  // motorCommand4 -> located at body -x +y
  REAR_LEFT = 4,
  // all motors as listed above
  ALL,
  // none of the motors
  NONE
};

//! Maps from a desired speed [rad/s] to a command output value
//  which is dimensionless
int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec);

//! Maps from a desired force [N] to a command speed [rad/s]
float speedFromForce(float desiredForce_N);

/**
 * @brief setMotorCommand sets the motor command for selected motors
 *
 * @param motorID the identifier of the motor(s) to be set
 * @param pwmCommand the desired pwm command to be set
 * @param out the output to be modified by this function
 */
void setMotorCommand(const MotorID motorID, const int pwmCommand, MainLoopOutput& out);
