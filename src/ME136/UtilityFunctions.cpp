#include "UtilityFunctions.hpp"
#include "UAVConstants.hpp"

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

std::tuple<float, float, float, float> mixToMotorForces(const float cSum, const float n1, const float n2, const float n3)
{
  // some precomputations for readability and to safe some computation time:
  const float lInv = 1 / Constants::UAV::propOrthDist;
  const float kInv = 1 / Constants::UAV::kappa;

  // formulating the mixer matrix as presented in the lecture as separate equations:
  // force of motor 1
  const float c1 = 0.25f * (1 +lInv*n1 -lInv*n2 +kInv*n3);
  const float c2 = 0.25f * (1 -lInv*n1 -lInv*n2 -kInv*n3);
  const float c3 = 0.25f * (1 -lInv*n1 +lInv*n2 +kInv*n3);
  const float c4 = 0.25f * (1 +lInv*n1 +lInv*n2 -kInv*n3);

  // stack the whole stuff to one tuple to return all in once
  return std::tuple<float, float, float, float>(c1, c2, c3, c4);
}
