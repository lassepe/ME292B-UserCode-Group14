/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.h: Time-of-flight distance sensor driver
 */

#pragma once

#include <stdint.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

class VL53L0_I2C : public device::I2C {
 public:
  VL53L0_I2C(int bus);
  virtual ~VL53L0_I2C();

  virtual int init();
  virtual int read(uint8_t address, void *data, unsigned count);
  virtual int write(unsigned address, void *data, unsigned count);

 protected:

};

class VL53L0X {
 public:
  VL53L0X();
  bool Init();
  void Loop();
  float GetRange()const {return _rangeLast;};
  void PrintStatus();

 private:
  // TCC: Target CentreCheck
  // MSRC: Minimum Signal Rate Check
  // DSS: Dynamic Spad Selection
  typedef struct {
    bool tcc;
    bool msrc;
    bool dss;
    bool pre_range;
    bool final_range;
  } SequenceStepEnables;

  typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
  } SequenceStepTimeouts;

  typedef enum vcselPeriodType_t {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
  } vcselPeriodType;

  /** Default constructor, uses external I2C address.
   * @see VL53L0X_DEFAULT_ADDRESS
   */

  bool vl53l0xTest(void);

  bool vl53l0xReadRange(float& zrangeOut);

  /** Verify the I2C connection.
   * Make sure the device is connected and responds as expected.
   * @return True if connection is valid, false otherwise
   */
  bool vl53l0xTestConnection();

  /** Get Model ID.
   * This register is used to verify the model number of the device,
   * but only before it has been configured to run
   * @return Model ID
   * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
   * @see VL53L0X_IDENTIFICATION_MODEL_ID
   */
  uint16_t vl53l0xGetModelID();

  /** Get Revision ID.
   * This register is used to verify the revision number of the device,
   * but only before it has been configured to run
   * @return Revision ID
   * @see VL53L0X_RA_IDENTIFICATION_REVISION_ID
   * @see VL53L0X_IDENTIFICATION_REVISION_ID
   */
  uint8_t vl53l0xGetRevisionID();

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
  bool vl53l0xInitSensor(bool io_2v8);

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
  bool vl53l0xSetSignalRateLimit(float limit_Mcps);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
  bool vl53l0xSetMeasurementTimingBudget(uint32_t budget_us);

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
  uint32_t vl53l0xGetMeasurementTimingBudget(void);

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
  bool vl53l0xSetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
  uint8_t vl53l0xGetVcselPulsePeriod(vcselPeriodType type);

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
  void vl53l0xStartContinuous(uint32_t period_ms);

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
  void vl53l0xStopContinuous(void);

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
  uint16_t vl53l0xReadRangeContinuousMillimeters(void);

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
  uint16_t vl53l0xReadRangeSingleMillimeters(void);

  // Get sequence step timeouts
  // based on get_sequence_step_timeout(),
  // but gets all timeouts instead of just the requested one, and also stores
  // intermediate values
  void vl53l0xGetSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

  // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
  // based on VL53L0X_calc_timeout_mclks()
  uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

  // Encode sequence step timeout register value from timeout in MCLKs
  // based on VL53L0X_encode_timeout()
  // Note: the original function took a uint16_t, but the argument passed to it
  // is always a uint16_t.
  uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks);

  // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
  // based on VL53L0X_calc_timeout_us()
  uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

  // Decode sequence step timeout in MCLKs from register value
  // based on VL53L0X_decode_timeout()
  // Note: the original function returned a uint32_t, but the return value is
  // always stored in a uint16_t.
  uint16_t vl53l0xDecodeTimeout(uint16_t reg_val);

  void vl53l0xGetSequenceStepEnables(SequenceStepEnables * enables);
  bool vl53l0xPerformSingleRefCalibration(uint8_t vhv_init_byte);
  uint16_t vl53l0xReadReg16Bit(uint8_t reg);
  bool vl53l0xWriteReg16Bit(uint8_t reg, uint16_t val);
  bool vl53l0xWriteReg32Bit(uint8_t reg, uint32_t val);


  bool vl53l0xGetSpadInfo(uint8_t * count, bool * type_is_aperture);

	VL53L0_I2C* _interface;
  uint8_t _devAddr;
  bool _isInit;
  float _rangeLast;
  unsigned _loopCount;
};

