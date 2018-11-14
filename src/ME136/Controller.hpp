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
   * @brief reset all states of the controller (e.g. integrator gains)
   */
  void reset() { integratedPositionError_ = {0, 0, 0}; }

  /**
   * @breief control perform the next step of the control task and call all
   * subroutine in the correct order.
   * @param in the inputs (measurements)
   * @return the four motor forces needed to perform the control task (c1, c2,
   * c3, c4)
   */
  std::tuple<float, float, float, float> control(
      const MainLoopInput& in, TelemetryLoggingInterface& logger) {
    // defining the set point

    // the set point for the position
    const Vec3f desiredPosition = {0, 0, 0.65f};
    Vec3f desAng = {0, 0, 0};
    float desiredThrust = 0;
    std::tie(desAng, desiredThrust) =
        controlTranslation(desiredPosition, logger);
    // controll the quad to a desired equilibrium attitude
    const Vec3f attitudeMotorTorques = controlAttitude(in, desAng, logger);

    // combine the commanded total thrust and desired motor torques and mix
    // them to the resulting thrusts per motor
    return mixToMotorForces(desiredThrust, attitudeMotorTorques);
  }

  /**
   * @brief controlTranslation controlls the translation of the quad and
   * resturns the desired angle and total thrust
   *
   * @return a tuple of the disired angle and the total thrust
   */
  std::tuple<Vec3f, float> controlTranslation(
      const Vec3f& desiredPosition, TelemetryLoggingInterface& logger) {
    // extracting the state estimate
    const auto attitudeEst = stateEstimation_.getAttitudeEst();
    const auto positionEst = stateEstimation_.getPositionEst();
    const auto velocityEst = stateEstimation_.getVelocityEst();

    // naming some constants for shorter code
    const float wn = Constants::Control::natFreq_height;
    const float d = Constants::Control::dampRat_height;

    Vec3f desVel = {0, 0, 0};
    const Vec3f positionError = (positionEst - desiredPosition);

    const float positionAccumulationGain = 0.001f;
    const float integratorMax = 1.f;
    const float integratorMin = -1.f;
    integratedPositionError_ += positionAccumulationGain * positionError;
    // limiting the integrator
    integratedPositionError_.x = std::min(std::max(integratedPositionError_.x, integratorMin), integratorMax);
    integratedPositionError_.y = std::min(std::max(integratedPositionError_.y, integratorMin), integratorMax);
    integratedPositionError_.z = std::min(std::max(integratedPositionError_.z, integratorMin), integratorMax);

    // Maybe log the desired velocity
    desVel = -1 / Constants::Control::timeConstant_position * positionError - integratedPositionError_;

    Vec3f desAcc = {0, 0, 0};

    const float pPos = .7f;
    const float iPos = 2.f;
    const float dPos = 3.f;

    desAcc.x = -pPos * positionError.x - iPos * integratedPositionError_.x - dPos * velocityEst.x;
    desAcc.y = -pPos * positionError.y - iPos * integratedPositionError_.y - dPos * velocityEst.y;
    desAcc.z = -2.f * d * wn * velocityEst.z - wn * wn * positionError.z;

    // for now we want to keep the angle of the quad always at 0
    Vec3f desAng = {0, 0, 0};
    desAng.x = -desAcc.y / Constants::World::gravity;
    desAng.y = desAcc.x / Constants::World::gravity;
    desAng.z = 0.f;

    // The low level controller
    // for now there is no deticated total thrust control but rather we only
    // set a constant value needed to nearly hover the quad
    const float denom = cosf(attitudeEst.x) * cosf(attitudeEst.y);
    const float cmdNormThrust =
        denom > 0.001f ? (Constants::World::gravity + desAcc.z) / denom
                       : 0.f * Constants::World::gravity;
    const float desiredThrust = cmdNormThrust * Constants::UAV::mass;

    // send all the relevant telemetry data
    // logger.log(desAng.x, "desAng.x");
    // logger.log(desAng.y, "desAng.y");
    logger.log(desVel.x, "desVel_x");
    logger.log(desVel.y, "desVel_y");
    logger.log(positionEst, "positionEst");

    return std::tuple<Vec3f, float>(desAng, desiredThrust);
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

  /// integrator state for the attitude controller
  Vec3f integratedPositionError_ = {0.f, 0.f, 0.f};
};
