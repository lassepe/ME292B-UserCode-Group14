#pragma once

#include "MainLoopTypes.hpp"
#include "Vec3f.hpp"

class SensorCalibration {
 public:
  /**
   * @brief SensorCalibration the constructor for this class. This class wraps away
   * the calibration routine. The user can specify over how many iterations he
   * wants to calibrate;
   * @param measurementsToRecord the number of measurements that should be used
   * for calibration.
   */
  SensorCalibration(const int measurementsToRecord)
      : finished_(false),
        numMeasurements_(0),
        measurementsToRecord_(measurementsToRecord),
        rateGyroOffset_(0, 0, 0) {}

  /// getters to savely access the internal state
  bool isFinished() const { return finished_; }
  int getNumMeasurements() const { return numMeasurements_; }
  const Vec3f& getRateGyroOffset() const { return rateGyroOffset_; }
  /// reset the calibration state back to 0
  void reset() {
    rateGyroOffset_ = Vec3f(0, 0, 0);
    finished_ = false;
    numMeasurements_ = 0;
  }

  /**
   * @brief run runs the calibration routine until we recorded enough
   * measurements
   * @param in the input of the main loop (containing the measurements)
   * @return returns true if this method is still running
   */
  bool run(const MainLoopInput& in) {
    // check if we still need to collect measurements
    if (!finished_) {
      // accumulate the rateGyroMeasurements
      rateGyroOffset_ =
          (rateGyroOffset_ * numMeasurements_ + in.imuMeasurement.rateGyro) /
          (numMeasurements_ + 1);
      numMeasurements_++;
    }

    // update the finished state
    finished_ = !(numMeasurements_ < measurementsToRecord_);
    // return if we are still running
    return !finished_;
  }

 private:
  /// true if the calibration phase is fished
  bool finished_ = false;
  /// the number of measurements recorded
  int numMeasurements_;
  /// the maximum number of measurements that we need to record
  const int measurementsToRecord_ = 500;

  /// the calibrated values
  Vec3f rateGyroOffset_ = Vec3f(0, 0, 0);
};
