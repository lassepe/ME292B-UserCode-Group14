#include "UtilityFunctions.hpp"

int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // the slope of the speedToPWM map
  const float m = 0.1118f;
  // the bias of the speedToPWMmap
  const float b = -66.637f;

  return int(b + m * desiredSpeed_rad_per_sec);
}


float speedFromForce(float desiredForce_N) {
  // replace this with your determined constant:
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 1.585e-8f;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}

void setMotorCommand(const MotorID motorID, const int pwmCommand, MainLoopOutput& out)
{
  switch(motorID)
  {
    case MotorID::ALL:
      out.motorCommand1 = pwmCommand;
      out.motorCommand2 = pwmCommand;
      out.motorCommand3 = pwmCommand;
      out.motorCommand4 = pwmCommand;
      break;
    case MotorID::FRONT_LEFT:
      out.motorCommand1 = pwmCommand;
      break;
    case MotorID::FRONT_RIGHT:
      out.motorCommand2 = pwmCommand;
      break;
    case MotorID::REAR_RIGHT:
      out.motorCommand3 = pwmCommand;
      break;
    case MotorID::REAR_LEFT:
      out.motorCommand4 = pwmCommand;
      break;
    default:
      break;
  }
}
