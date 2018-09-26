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
  float const propConstant = 1.0e-08f;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}

