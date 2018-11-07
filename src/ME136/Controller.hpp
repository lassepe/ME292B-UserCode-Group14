#pragma once

#include <cmath>
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
    // extracting the state estimate
    const auto velocityEst = stateEstimation_.getVelocityEst();
    const auto heightEst = stateEstimation_.getHeightEst();
    const auto attitudeEst = stateEstimation_.getAttitudeEst();
    const auto poseEst = stateEstimation_.getPoseEst();
    // defining the set point
    const float desHeight = 1.f;
    // naming some constants for shorter code
    const float wn = Constants::Control::natFreq_height;
    const float d = Constants::Control::dampRat_height;

    Vec3f desVel = {0, 0, 0};
    const float positionTimeConstant = 5.f;
    desVel = -1 / positionTimeConstant * (poseEst);

    Vec3f desAcc = {0, 0, 0};
    desAcc.x = -1 / Constants::Control::timeConstant_horizVel * (velocityEst.x  - desVel.x);
    desAcc.y = -1 / Constants::Control::timeConstant_horizVel * (velocityEst.y - desVel.y);
    desAcc.z =
        -2.f * d * wn * velocityEst.z - wn * wn * (heightEst - desHeight);
    // for now we want to keep the angle of the quad always at 0
    Vec3f desAng = {0, 0, 0};
    desAng.x = -desAcc.y / Constants::World::gravity;
    desAng.y = desAcc.x / Constants::World::gravity;
    desAng.z = 0.f;

    // controll the quad to a desired equilibrium attitude
    const Vec3f attitudeMotorTorques = controlAttitude(in, desAng, logger);

    // for now there is no deticated total thrust control but rather we only
    // set a constant value needed to nearly hover the quad
    const float denom = cosf(attitudeEst.x) * cosf(attitudeEst.y);
    const float cmdNormThrust =
        denom > 0.001f ? (Constants::World::gravity + desAcc.z) / denom
                       : 0.f * Constants::World::gravity;
    const float desiredThrust = cmdNormThrust * Constants::UAV::mass;

    // send all the relevant telemetry data
    logger.log(desAng.x, "desAng.x");
    logger.log(desAng.y, "desAng.y");
    logger.log(desiredThrust, "desiredThrust");

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

    // outer control loop: Figure out the angular velocity command to stay
    // stable
    Vec3f cmdAngVel;
    cmdAngVel.x =
        -(eulerEst.x - desAng.x) / Constants::Control::timeConstant_rollAngle;
    cmdAngVel.y =
        -(eulerEst.y - desAng.y) / Constants::Control::timeConstant_pitchAngle;
    cmdAngVel.z =
        -(eulerEst.z - desAng.z) / Constants::Control::timeConstant_yawAngle;

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
                               TelemetryLoggingInterface& /*logger*/) {
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

    return Vec3f(n1, n2, n3);
  }

 private:
  /// a reference to the calibration results
  const SensorCalibration& sensorCalibration_;
  /// a reference to the estimated states
  const StateEstimation& stateEstimation_;
};
