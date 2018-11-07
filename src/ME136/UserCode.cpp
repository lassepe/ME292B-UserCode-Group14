#include "UserCode.hpp"
#include "UserInputHandler.hpp"
#include "SensorCalibration.hpp"
#include "StateEstimation.hpp"
#include "Controller.hpp"
#include "TelemetryLoggingInterface.hpp"
#include "UAVConstants.hpp"
#include "Vec3f.hpp"

#include <stdio.h>  //for printf

// We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// the calibration module of the system, handling the calibration of the
// sensors (for details see documentation within the class)
SensorCalibration sensorCalibration = SensorCalibration(500);
// the state estimation module of the system, handling the estimation of the
// current state. This needs to be created AFTER the calibration at it makes
// use of the calibrated offsets.
StateEstimation stateEstimation = StateEstimation(0.01, 0.3f, sensorCalibration);
// the Controller computes thrust forces at every time step. The controller
// takes a constant reference to the calibration and to the state estimation to
// have access to corrected measurement and the whole estimated state
Controller controller = Controller(sensorCalibration, stateEstimation);
// some space to store the last channel info (needed for priting)
std::vector<const char*> lastChannelInfo;

MainLoopOutput MainLoop(MainLoopInput const& in) {
  // the main loop out
  MainLoopOutput out;
  // a logger to conveniently add telemtry logs to out
  auto logger = TelemetryLoggingInterface(12, out);

  if (in.joystickInput.buttonStart) {
    // if someone hit the start button we reset the calibration and the state
    // estimation
    sensorCalibration.reset();
    stateEstimation.reset();
  }
  // if the calibration is still running we have to return early
  if (!sensorCalibration.run(in)) {
    // only do things if the calibration is finished.
    stateEstimation.update(in, logger);
  }

  // compute the 4 motor torques from the control policy
  float c1, c2, c3, c4;
  std::tie(c1, c2, c3, c4) = controller.control(in, logger);
  // finally set the torques
  setMotorCommand(MotorID(1), pwmFromForce(c1), out);
  setMotorCommand(MotorID(2), pwmFromForce(c2), out);
  setMotorCommand(MotorID(3), pwmFromForce(c3), out);
  setMotorCommand(MotorID(4), pwmFromForce(c4), out);

  // copy the inputs and outputs for printing
  lastChannelInfo = logger.getChannelInfo();
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
    printf("================================================");
    printf("\n");
    printf("Last_range_=_%6.3fm,_", \
           double(lastMainLoopInputs.heightSensor.value));
    printf("Last_flow:_x=%6.3,_y=%6.3f\n", \
           double(lastMainLoopInputs.opticalFlowSensor.value_x), \
           double(lastMainLoopInputs.opticalFlowSensor.value_y));
    printf("\n");
    printf("Telemetry Channel Info");
    printf("\n");
    int i = 0;
    for (auto info : lastChannelInfo)
    {
      printf("Channel %d: ", (i));
      printf(info);
      printf(" = %6.3f", double(lastMainLoopOutputs.telemetryOutputs_plusMinus100[i]));
      printf("\n");
      i++;
    }
    printf("================================================");
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
  printf("timeConstant_pitchAngle:%6.3f",double(Constants::Control::timeConstant_pitchAngle));
}
