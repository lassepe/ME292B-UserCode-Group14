#pragma once

#include <tuple>

#include "MainLoopTypes.hpp"
#include "SensorCalibration.hpp"
#include "StateEstimation.hpp"
#include "TelemetryLoggingInterface.hpp"
#include "UAVConstants.hpp"
#include "UtilityFunctions.hpp"

class Controller {
 public:
  /**
   * @brief Controller computes the motor commands from the estimated states
   * and a given reference
   */
  Controller(const SensorCalibration& sensorCalibration,
             const StateEstimation& stateEstimation)
      : sensorCalibration_(sensorCalibration),
        stateEstimation_(stateEstimation) {}

  /**
   * @breief control perform the next step of the control task and call all
   * subroutine in the correct order.
   * @param in the inputs (measurements)
   * @return the four motor forces needed to perform the control task (c1, c2,
   * c3, c4)
   */
  std::tuple<float, float, float, float> control(
      const MainLoopInput& in, TelemetryLoggingInterface& logger) {
    // for now we want to keep the angle of the quad always at 0
    Vec3f desAng = {0, 0, 0};
    // step in put experiment
    if (in.joystickInput.buttonBlue) {
      desAng.y = 0.5236f;
    }
    // controll the quad to a desired equilibrium attitude
    const Vec3f attitudeMotorTorques = controlAttitude(in, desAng, logger);

    // for now there is no deticated total thrust control but rather we only
    // set a constant value needed to nearly hover the quad
    const float cmdNormThrust = 2.0f;
    float desiredThrust = cmdNormThrust * Constants::UAV::mass;
    // combine the commanded total thrust and desired motor torques and mix
    // them to the resulting thrusts per motor
    return mixToMotorForces(desiredThrust, attitudeMotorTorques);
  }

  /**
   * @brief the outer controll loop that  performs the attitude control
   * @param in a reference to the measurements input to the controller
   * @param desAng the desired set point we want to reach
   * @return the motor torques that need be commanded in order to perform the
   * control task
   */
  Vec3f controlAttitude(const MainLoopInput& in, const Vec3f& desAng,
                        TelemetryLoggingInterface& logger) {
    // extract the corrected measurements needed for control
    const auto eulerEst = stateEstimation_.getAttitudeEst();

    // outer control loop: Figure out the angular velocity command to stay stable
    Vec3f cmdAngVel;
    cmdAngVel.x =
        -(eulerEst.x - desAng.x) / Constants::Control::timeConstant_rollAngle;
    cmdAngVel.y =
        -(eulerEst.y - desAng.y) / Constants::Control::timeConstant_pitchAngle;
    cmdAngVel.z =
        -(eulerEst.z - desAng.z) / Constants::Control::timeConstant_yawAngle;

    // log the data for plotting
    logger.log(desAng.y, "desAng.y");
    logger.log(cmdAngVel, "desAngVel");
    logger.log(eulerEst, "eulerEst");

    // call the inner control loop and return its values
    return controlAngularVelocity(in, cmdAngVel, logger);
  }

  /**
   * @brief controlAngularVelocity perform the control task of controlling the
   * quad to a certain desired angular velocity
   * @param in a reference to the inputs (measurements)
   * @param desAngVel the desired angular velocity that we want to achieve
   */
  Vec3f controlAngularVelocity(const MainLoopInput& in, const Vec3f& desAngVel,
                               TelemetryLoggingInterface& logger) {
    // extract the corrected measurements needed for control
    const auto gyroCalibrated =
        in.imuMeasurement.rateGyro - sensorCalibration_.getRateGyroOffset();


    Vec3f cmdAngAcc;

    cmdAngAcc.x = -(gyroCalibrated.x - desAngVel.x) /
                  Constants::Control::timeConstant_rollRate;
    cmdAngAcc.y = -(gyroCalibrated.y - desAngVel.y) /
                  Constants::Control::timeConstant_pitchRate;
    cmdAngAcc.z = -(gyroCalibrated.z - desAngVel.z) /
                  Constants::Control::timeConstant_yawRate;

    float n1 = cmdAngAcc.x * Constants::UAV::inertia_xx;
    float n2 = cmdAngAcc.y * Constants::UAV::inertia_yy;
    float n3 = cmdAngAcc.z * Constants::UAV::inertia_zz;

    // log the data for plotting
    logger.log(gyroCalibrated.y, "gyroCalibrated.y");
    logger.log(cmdAngAcc, "cmdAngAcc");

    return Vec3f(n1, n2, n3);
  }

 private:
  /// a reference to the calibration results
  const SensorCalibration& sensorCalibration_;
  /// a reference to the estimated states
  const StateEstimation& stateEstimation_;
};
