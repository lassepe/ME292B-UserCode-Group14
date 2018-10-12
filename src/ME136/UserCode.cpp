#include "UserCode.hpp"
#include "SensorCalibration.hpp"
#include "StateEstimation.hpp"
#include "UAVConstants.hpp"
#include "Vec3f.hpp"

#include <stdio.h>  //for printf

// We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// a namespace to wrap the scope of some state variables
namespace UserInputState {
// Some state to cycle through the different PWM values
int currentPWMIndex = 0;
// Some state to cycle through the different speed values
int currentSpeedIndex = 0;
// state to figure out which button was used
bool resetButtonWasPressed = false;
// the pwm values the user can select from
const int desiredPWM[6] = {40, 80, 120, 160, 200, 240};
// the speed values the user can select from
const int desiredSpeed[4] = {1000, 1200, 1400, 1600};
};  // namespace UserInputState

// the calibration module of the system, handling the calibration of the
// sensors (for details see documentation within the class)
SensorCalibration sensorCalibration = SensorCalibration(500);
// the state estimation module of the system, handling the estimation of the
// current state. This needs to be created AFTER the calibration at it makes
// use of the calibrated offsets.
StateEstimation stateEstimation = StateEstimation(0.01, sensorCalibration);

void updateInputState(const MainLoopInput& in) {
  // if the reset button was pressed before we can set a new value
  if (in.joystickInput.buttonGreen) {
    // green is our reset button which has to be pressed before we can
    // increment
    UserInputState::resetButtonWasPressed = true;
  } else if (UserInputState::resetButtonWasPressed &&
             in.joystickInput.buttonYellow) {
    UserInputState::currentPWMIndex++;
    UserInputState::currentSpeedIndex++;
    // wrap around
    UserInputState::currentPWMIndex %= 6;
    UserInputState::currentSpeedIndex %= 4;
    UserInputState::resetButtonWasPressed = false;
  }
}

const MainLoopOutput userSetDesiredSpeed(const MainLoopInput& in,
                                         const MotorID motorID) {
  MainLoopOutput out;
  // cycling through the different rpm values
  if (in.joystickInput.buttonBlue) {
    const int pwmCommand = pwmCommandFromSpeed(
        UserInputState::desiredSpeed[UserInputState::currentSpeedIndex]);
    setMotorCommand(motorID, pwmCommand, out);
  }
  return out;
}

const MainLoopOutput userSetDesiredPWMCommand(const MainLoopInput& in,
                                              const MotorID motorID) {
  MainLoopOutput out;
  // only set the value of the blue button is pressed
  if (in.joystickInput.buttonBlue) {
    // select the desired pwm signal
    const int pwmCommand =
        UserInputState::desiredPWM[UserInputState::currentPWMIndex];
    // set the desired speed for the motors as specified
    setMotorCommand(motorID, pwmCommand, out);
  }
  return out;
}

MainLoopOutput MainLoop(MainLoopInput const& in) {
  // the main loop out
  MainLoopOutput out;
  if (in.joystickInput.buttonStart) {
    // if someone hit the start button we reset the calibration and the state
    // estimation
    sensorCalibration.reset();
    stateEstimation.reset();
  }
  // if the calibration is still running we have to return early
  if (!sensorCalibration.run(in)) {
    // only do things if the calibration is finished.
    stateEstimation.update(in, UAVConstants::dt);
  }

  // get the current attitude estimate to send it via telemetry
  const auto eulerEst = stateEstimation.getAttitudeEst();
  out.telemetryOutputs_plusMinus100[0] = eulerEst.x;
  out.telemetryOutputs_plusMinus100[1] = eulerEst.y;
  out.telemetryOutputs_plusMinus100[2] = eulerEst.z;

  const auto gyroCalibrated = lastMainLoopInputs.imuMeasurement.rateGyro -
                              sensorCalibration.getRateGyroOffset();
  out.telemetryOutputs_plusMinus100[3] = gyroCalibrated.x;
  out.telemetryOutputs_plusMinus100[4] = gyroCalibrated.y;
  out.telemetryOutputs_plusMinus100[5] = gyroCalibrated.z;
  // copy the inputs and outputs for printing
  lastMainLoopInputs = in;
  lastMainLoopOutputs = out;
  return out;
}

void PrintStatus() {
  // For a quick reference on the printf function, see:
  // http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it
  // to
  //  `double` in the printf function, and explicitly specify precision using
  //  something like %6.3f (six significant digits, three after the period).
  //  Example:
  //   printf("  exampleVariable_float = %6.3f\n",
  //   double(exampleVariable_float));

  // Accelerometer measurement

  if (!sensorCalibration.isFinished()) {
    printf("================================================");
    printf("\n");
    printf(">> Calibrating! Please Wait. Don't move the UAV.");
    printf("\n");
    printf("================================================");
    printf("\n");
  } else {
    printf("Acc: ");
    printf("x=%6.3f, ",
           double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
    printf("\n");
    printf("================================================");
    printf("\n");
    printf("Gyro: ");
    printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
    printf("\n");
    printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
    printf("\n");
    printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
    printf("\n");
    printf("================================================");
    printf("\n");
    printf("GyroCalibrated: ");
    printf("\n");
    const auto gyroCalibrated = lastMainLoopInputs.imuMeasurement.rateGyro -
                                sensorCalibration.getRateGyroOffset();
    printf("rollRateCalibrated=%6.3f, ", double(gyroCalibrated.x));
    printf("\n");
    printf("pitchRateCalibrated=%6.3f, ", double(gyroCalibrated.y));
    printf("\n");
    printf("yawRateCalibrated=%6.3f, ", double(gyroCalibrated.z));
    printf("\n");
    printf("================================================");
    printf("\n");
    printf("AttitudeEstimation:");
    printf("\n");
    const auto eulerEst = stateEstimation.getAttitudeEst();
    printf("rollEst=%6.3f, ", double(eulerEst.x));
    printf("\n");
    printf("pitchEst=%6.3f, ", double(eulerEst.y));
    printf("\n");
    printf("yawEst=%6.3f, ", double(eulerEst.z));
    printf("\n");
  }

  // Note that it is somewhat annoying to print float variables.
  // We need to cast the variable as double, and we need to specify
  // the number of digits we want (if you used simply "%f", it would
  // truncate to an integer.
  // Here, we print 6 digits, with three digits after the period.
  // printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));
  // just an example of how we would inspect the last main loop inputs and
  // outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed) printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen) printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue) printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow) printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart) printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect) printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor command 1 = %6.3f\n",
         double(lastMainLoopOutputs.motorCommand1));
}
