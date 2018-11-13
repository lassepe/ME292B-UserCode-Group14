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
  StateEstimation(const SensorCalibration& sensorCalibration)
      : rhoAttitude_(Constants::StateEstimation::rhoAttitude),
        rhoVertical_(Constants::StateEstimation::rhoVertical),
        rhoHorizontalVel_(Constants::StateEstimation::rhoHorizontalVel),
        sensorCalibration_(sensorCalibration) {}

  void reset() {
    attitudeEst_ = Vec3f(0, 0, 0);
    heightEst_ = 0.f;
    velocityEst_ = Vec3f(0, 0, 0);
    lastHeightMeas_ = 0.f;
    lastHeightMeasTime_ = 0.f;
    positionEstimate_ = {0, 0, 0};
  }
  /**
   * @brief getAttitudeEst a getter for the estimated attitude (this is only
   * valid for small angles and accelerations)
   * @return the current estimate for the attitude of the system as euler
   * angles.
   */
  const Vec3f& getAttitudeEst() const { return attitudeEst_; }

  /// getters for all internal estimates
  const Vec3f& getVelocityEst() const { return velocityEst_; }
  float getHeightEst() const { return heightEst_; }
  const Vec3f& getPositionEst() const { return positionEstimate_; }

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
    // update the vertical translational estimator
    updateVerticalEst(in.heightSensor.value, in.heightSensor.updated,
                      in.currentTime);
    // update the horizontal translational estimator
    updateHorizontalEst(in.opticalFlowSensor.value_x,
                        in.opticalFlowSensor.value_y,
                        in.opticalFlowSensor.updated, rateGyroMeasCorrected);

    // update the position estiamtion
    updatePositionEstimation();

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
  const float rhoVertical_ = 0.3f;
  /// the estimated velocities in all 3 directions:
  Vec3f velocityEst_ = Vec3f(0.f, 0.f, 0.f);
  /// the filter gain for the horizontal velocity
  const float rhoHorizontalVel_ = 0.1f;
  /// states of the last cycle:
  // TODO: maybe use the difference to the last state not the last measurement
  float lastHeightMeas_ = 0.f;
  float lastHeightMeasTime_ = 0.f;
  /// the estimated pose
  Vec3f positionEstimate_ = {0, 0, 0};

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
      heightEst_ = (1 - rhoVertical_) * heightEst_ + rhoVertical_ * heightMeas;

      // update the vectical velocity estimate
      // TODO: maybe use diff to height state here
      const float velocityVecticalMeas =
          (heightMeas - lastHeightMeas_) / (updateTime - lastHeightMeasTime_);
      // update the vertical velocity estimate
      // TODO: maybe use another filter constant here
      velocityEst_.z = (1 - rhoVertical_) * velocityEst_.z +
                       rhoVertical_ * velocityVecticalMeas;

      // update the state to compute the finite difference in the next steps
      lastHeightMeas_ = heightMeas;
      lastHeightMeasTime_ = updateTime;
    }
  }

  void updateHorizontalEst(const float sigma1, const float sigma2,
                           const bool updated,
                           const Vec3f& rateGyroMeasCorrected) {
    if (updated) {
      const float denom = cosf(attitudeEst_.x) * cosf(attitudeEst_.y);
      if (denom > 0.5f) {
        const float deltaPredict = heightEst_ / denom;
        // creating the pseudo measurements
        const float vXMeas = (-sigma1 + rateGyroMeasCorrected.y) * deltaPredict;
        const float vYMeas = (-sigma2 - rateGyroMeasCorrected.x) * deltaPredict;

        // update the estimates
        velocityEst_.x = (1 - rhoHorizontalVel_) * velocityEst_.x +
                         rhoHorizontalVel_ * vXMeas;
        velocityEst_.y = (1 - rhoHorizontalVel_) * velocityEst_.y +
                         rhoHorizontalVel_ * vYMeas;
      }
    }
  }

  void updatePositionEstimation() {
    positionEstimate_.x += (cosf(attitudeEst_.z) * velocityEst_.x -
                   sinf(attitudeEst_.z) * velocityEst_.y) *
                  Constants::UAV::dt;
    positionEstimate_.y += (cosf(attitudeEst_.z) * velocityEst_.y +
                   sinf(attitudeEst_.z) * velocityEst_.x) *
                  Constants::UAV::dt;
    positionEstimate_.z = heightEst_;
  }
};
