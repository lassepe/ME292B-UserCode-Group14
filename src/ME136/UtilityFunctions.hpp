#pragma once

#include <tuple>
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

/**
 * @brief mixToMotorForces mixes the disired total force and the three desired torques to yield the three motor forces
 *
 * @param cSum the total force desired
 * @param n1 the torque around axis the B-1-axis
 * @param n2 the torque around axis the B-2-axis
 * @param n3 the torque around axis the B-3-axis
 *
 * @return a tuple of the resulting motor forces c1, c2, c3, c4
 */
std::tuple<float, float, float, float> mixToMotorForces(const float cSum, const float n1, const float n2, const float n3);
