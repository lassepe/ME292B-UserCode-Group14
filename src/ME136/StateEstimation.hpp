#pragma once

#include "MainLoopTypes.hpp"
#include "SensorCalibration.hpp"
#include "TelemetryLoggingInterface.hpp"
#include "UAVConstants.hpp"
#include "Vec3f.hpp"

/**
 * @brief StateEstimation is a simple class to wrap away the state estimation
 * into a self-contained module. This way, changing details inside the state
 * estimation throught the next labs should avoid having to change things all
 * over the place.
 */
class StateEstimation {
 public:
  /**
   * @brief the constructor of this class. The estimator wraps away the whole
   * part of state esimation.
   * @param rhoAttitude the filter parameter for the attitude filter
   * @param a reference to the sensorCalibration module
   */
  StateEstimation(const float rhoAttitude,
                  const SensorCalibration& sensorCalibration)
      : attitudeEst_(0, 0, 0),
        rhoAttitude_(rhoAttitude),
        sensorCalibration_(sensorCalibration) {}

  void reset() { attitudeEst_ = Vec3f(0, 0, 0); }
  /**
   * @brief getAttitudeEst a getter for the estimated attitude (this is only
   * valid for small angles and accelerations)
   * @return the current estimate for the attitude of the system as euler
   * angles.
   */
  const Vec3f& getAttitudeEst() const { return attitudeEst_; }

  /**
   * @brief updates the estimator with the current measurements
   * @param in the intput that contains all the measurements
   * @param dt the temporal difference to the last update
   */
  void update(const MainLoopInput& in, const float dt, TelemetryLoggingInterface& logger) {
    // calculate the corrected rate gyro value
    const auto rateGyroMeasCorrected =
        in.imuMeasurement.rateGyro - sensorCalibration_.getRateGyroOffset();
    // update the attitude estimator
    updateAttitudeEst(rateGyroMeasCorrected, in.imuMeasurement.accelerometer,
                      dt);

    // log all the relevant estimates
    logger.log(attitudeEst_, "attitudeEst_");
  }

 private:
  /// the current estimate of the filter for te attitue
  Vec3f attitudeEst_ = Vec3f(0, 0, 0);
  /// the filter parameter for the rate gyro
  const float rhoAttitude_ = 0.01f;
  /// a reference to the rate gyro offset (for sensorCalibration)
  const SensorCalibration& sensorCalibration_;

  /**
   * @brief updateAttitudeEst updates the attitude estimator with a simple
   * predictor-corrector method.
   * @param dt the temporal difference to the last update
   */
  void updateAttitudeEst(const Vec3f& rateGyroMeasCorrected,
                         const Vec3f& accMeas, const float dt) {
    // create hte measurements for roll and pitch
    const float rollMeas = accMeas.y / Constants::World::gravity;
    const float pitchMeas = -accMeas.x / Constants::World::gravity;

    // for the roll and the pitch angle we can use the accelormeter to correct
    // the prediction
    attitudeEst_.x =
        (1 - rhoAttitude_) * (attitudeEst_.x + rateGyroMeasCorrected.x * dt) +
        rhoAttitude_ * rollMeas;
    attitudeEst_.y =
        (1 - rhoAttitude_) * (attitudeEst_.y + rateGyroMeasCorrected.y * dt) +
        rhoAttitude_ * pitchMeas;
    // for the yaw we can just integrate (this is technically useless but that
    // is the best guess that we have until we can get other measurements to
    // correct yaw (e.g. visual reference or compass))
    attitudeEst_.z += rateGyroMeasCorrected.z * dt;
  }
};
