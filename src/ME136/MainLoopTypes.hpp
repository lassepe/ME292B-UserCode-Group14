#pragma once

#include <stdint.h>

#include "Vec3f.hpp"//our standard vector type

/*!
 * This defines some types we use for the main loop.
 */
struct MainLoopInput {
  float currentTime;  //[s]

  struct {
    float value;  //[V]
    bool updated;
  } batteryVoltage;

  struct {
    Vec3f accelerometer;  //[m/s**2]
    Vec3f rateGyro;  //[rad/s]
    bool updated;
  } imuMeasurement;

  struct {
    float axisLeftVertical;  //no units
    float axisLeftHorizontal;  //no units
    float axisRightVertical;  //no units
    float axisRightHorizontal;  //no units
    bool buttonRed;
    bool buttonGreen;
    bool buttonBlue;
    bool buttonYellow;
    bool buttonStart;
    bool buttonSelect;
    bool updated;
  } joystickInput;

  struct {
    int32_t value_x;
    int32_t value_y;
    bool updated;
  } opticalFlowSensor;

  struct {
    float value;  //[m]
    bool updated;
  } heightSensor;

};

struct MainLoopOutput {
  int motorCommand1;  // located at body +x +y
  int motorCommand2;  // located at body +x -y
  int motorCommand3;  // located at body -x -y
  int motorCommand4;  // located at body -x +y

  bool led1;
  bool led2;
  bool led3;
  bool led4;

  //variables that are only used for telemetry:
  float telemetryOutputs_plusMinus100[12];  // NOTE! These are bounded to be in +/- 100
  uint8_t telemetryOutputDebugChar;
};
