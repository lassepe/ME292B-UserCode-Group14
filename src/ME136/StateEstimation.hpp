#pragma once

#include <cmath>
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
  StateEstimation(const float rhoAttitude, const float rhoHeight,
                  const SensorCalibration& sensorCalibration)
      : attitudeEst_(0, 0, 0),
        rhoAttitude_(rhoAttitude),
        rhoHeight_(rhoHeight),
        sensorCalibration_(sensorCalibration) {}

  void reset() {
    attitudeEst_ = Vec3f(0, 0, 0);
    heightEst_ = 0.f;
    velocityEst_ = Vec3f(0, 0, 0);
    lastHeightMeas_ = 0.f;
    lastHeightMeasTime_ = 0.f;
  }
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
  void update(const MainLoopInput& in, TelemetryLoggingInterface& logger) {
    // calculate the corrected rate gyro value
    const auto rateGyroMeasCorrected =
        in.imuMeasurement.rateGyro - sensorCalibration_.getRateGyroOffset();
    // update the attitude estimator
    updateAttitudeEst(rateGyroMeasCorrected, in.imuMeasurement.accelerometer);
    // update the translational estimator
    updateVerticalEst(in.heightSensor.value, in.heightSensor.updated,
                      in.currentTime);

    // log all the relevant estimates
    logger.log(attitudeEst_, "attitudeEst_");
    logger.log(velocityEst_, "velocityEst_");
    logger.log(heightEst_, "heightEst_");
  }

 private:
  /// the current estimate of the filter for te attitue
  Vec3f attitudeEst_ = Vec3f(0, 0, 0);
  /// the filter parameter for the rate gyro
  const float rhoAttitude_ = 0.01f;

  /// the estiamted height of the quad
  float heightEst_ = 0.f;
  /// the filter gain for the height
  const float rhoHeight_ = 0.3f;
  /// the estimated velocities in all 3 directions:
  Vec3f velocityEst_ = Vec3f(0.f, 0.f, 0.f);
  /// states of the last cycle:
  // TODO: maybe use the difference to the last state not the last measurement
  float lastHeightMeas_ = 0.f;
  float lastHeightMeasTime_ = 0.f;

  /// a reference to the rate gyro offset (for sensorCalibration)
  const SensorCalibration& sensorCalibration_;

  /**
   * @brief updateAttitudeEst updates the attitude estimator with a simple
   * predictor-corrector method.
   * @param dt the temporal difference to the last update
   */
  void updateAttitudeEst(const Vec3f& rateGyroMeasCorrected,
                         const Vec3f& accMeas) {
    // create hte measurements for roll and pitch
    const float rollMeas = accMeas.y / Constants::World::gravity;
    const float pitchMeas = -accMeas.x / Constants::World::gravity;

    // for the roll and the pitch angle we can use the accelormeter to correct
    // the prediction
    attitudeEst_.x =
        (1 - rhoAttitude_) *
            (attitudeEst_.x + rateGyroMeasCorrected.x * Constants::UAV::dt) +
        rhoAttitude_ * rollMeas;
    attitudeEst_.y =
        (1 - rhoAttitude_) *
            (attitudeEst_.y + rateGyroMeasCorrected.y * Constants::UAV::dt) +
        rhoAttitude_ * pitchMeas;
    // for the yaw we can just integrate (this is technically useless but that
    // is the best guess that we have until we can get other measurements to
    // correct yaw (e.g. visual reference or compass))
    attitudeEst_.z += rateGyroMeasCorrected.z * Constants::UAV::dt;
  }

  void updateVerticalEst(const float heightSenseValue, const bool updated,
                         const float updateTime) {
    // prediction:
    heightEst_ += velocityEst_.z * Constants::UAV::dt;

    // correction step
    const float heightRejectionThreshold = 5.f;
    if (updated && heightSenseValue < heightRejectionThreshold) {
      // the atttiude corrected height measurement
      const float heightMeas =
          heightSenseValue * cosf(attitudeEst_.x) * cosf(attitudeEst_.y);
      // update the height estimate
      heightEst_ = (1 - rhoHeight_) * heightEst_ + rhoHeight_ * heightMeas;

      // update the vectical velocity estimate
      // TODO: maybe use diff to height state here
      const float velocityVecticalMeas =
          (heightMeas - lastHeightMeas_) / (updateTime - lastHeightMeasTime_);
      // update the vertical velocity estimate
      // TODO: maybe use another filter constant here
      velocityEst_.z =
          (1 - rhoHeight_) * velocityEst_.z + rhoHeight_ * velocityVecticalMeas;

      // update the state to compute the finite difference in the next steps
      lastHeightMeas_ = heightMeas;
      lastHeightMeasTime_ = updateTime;
    }
  }
};
