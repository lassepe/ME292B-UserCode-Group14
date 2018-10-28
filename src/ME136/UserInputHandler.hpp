#pragma once

#include "MainLoopTypes.hpp"
#include "UtilityFunctions.hpp"

class UserInputHandler {
 public:
  UserInputHandler(){};

  /**
   *
   * @brief updateInputState updates the state variables of the user inut (e.g.
   * for debouncing)
   * @param in the input of the user
   */
  void updateInputState(const MainLoopInput& in) {
    // if the reset button was pressed before we can set a new value
    if (in.joystickInput.buttonGreen) {
      // green is our reset button which has to be pressed before we can
      // increment
      resetButtonWasPressed_ = true;
    } else if (resetButtonWasPressed_ && in.joystickInput.buttonYellow) {
      currentPWMIndex_++;
      currentSpeedIndex_++;
      // wrap around
      currentPWMIndex_ %= 6;
      currentSpeedIndex_ %= 4;
      resetButtonWasPressed_ = false;
    }
  }
  /**
   * @brief userSetDesiredSpeed lets the user pick from a set of speeds and
   * cycle through them using the yellow and green button.
   * @param in the input of the user
   * @return the motor commands as MainLoopOutput that should reach the desired
   * speed selected by the user.
   */
  const MainLoopOutput userSetDesiredSpeed(const MainLoopInput& in,
                                           const MotorID motorID) {
    MainLoopOutput out;
    // cycling through the different rpm values
    if (in.joystickInput.buttonBlue) {
      const int pwmCommand =
          pwmCommandFromSpeed(desiredSpeed_[currentSpeedIndex_]);
      setMotorCommand(motorID, pwmCommand, out);
    }
    return out;
  }
  /**
   * @brief userSetDesiredPWMCommand lets the user pick from a set of PWM signals
   * and cycle through them using the yellow and green button.
   * @param in the input of the user
   * @param motorID the id of the motor that we would like to control
   * @return the motor commands as selected by the user
   */
  const MainLoopOutput userSetDesiredPWMCommand(const MainLoopInput& in,
                                                const MotorID motorID) {
    MainLoopOutput out;
    // only set the value of the blue button is pressed
    if (in.joystickInput.buttonBlue) {
      // select the desired pwm signal
      const int pwmCommand = desiredPWM_[currentPWMIndex_];
      // set the desired speed for the motors as specified
      setMotorCommand(motorID, pwmCommand, out);
    }
    return out;
  }

 private:
  // Some state to cycle through the different PWM values
  int currentPWMIndex_ = 0;
  // Some state to cycle through the different speed values
  int currentSpeedIndex_ = 0;
  // state to figure out which button was used
  bool resetButtonWasPressed_ = false;
  // the pwm values the user can select from
  const int desiredPWM_[6] = {40, 80, 120, 160, 200, 240};
  // the speed values the user can select from
  const int desiredSpeed_[4] = {1000, 1200, 1400, 1600};
};
