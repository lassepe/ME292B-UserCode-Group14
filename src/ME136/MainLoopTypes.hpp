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
  float motorCommand1;  //[rad/s]
  float motorCommand2;  //[rad/s]
  float motorCommand3;  //[rad/s]
  float motorCommand4;  //[rad/s]

  bool led1;
  bool led2;
  bool led3;
  bool led4;
};
