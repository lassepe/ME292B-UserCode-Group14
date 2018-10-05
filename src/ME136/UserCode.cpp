#include "UserCode.hpp"
#include "Vec3f.hpp"
#include "UAVConstants.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// a namespace to wrap the scope of some state variables
namespace UserInputState {
  // Some state to cycle through the different PWM values
  int currentPWMIndex = 0;
  // Some state to cycle through the different speed values
  int currentSpeedIndex = 0;
  // state to figure out which button was used
  bool resetButtonWasPressed = false;
  // the pwm values the user can select from
  const int desiredPWM[6] = {40, 80, 120, 160, 200, 240};
  // the speed values the user can select from
  const int desiredSpeed[4] = {1000, 1200, 1400, 1600};
};

void updateInputState(const MainLoopInput& in) {
  // if the reset button was pressed before we can set a new value
  if (in.joystickInput.buttonGreen)
  {
    // green is our reset button which has to be pressed before we can
    // increment
    UserInputState::resetButtonWasPressed = true;
  }
  else if (UserInputState::resetButtonWasPressed && in.joystickInput.buttonYellow)
  {
    UserInputState::currentPWMIndex++;
    UserInputState::currentSpeedIndex++;
    // wrap around
    UserInputState::currentPWMIndex %= 6;
    UserInputState::currentSpeedIndex %= 4;
    UserInputState::resetButtonWasPressed = false;
  }
}

const MainLoopOutput userSetDesiredSpeed(const MainLoopInput& in,
                                         const MotorID motorID) {
  MainLoopOutput out;
  // cycling through the different rpm values
  if (in.joystickInput.buttonBlue) {
    const int pwmCommand = pwmCommandFromSpeed(
        UserInputState::desiredSpeed[UserInputState::currentSpeedIndex]);
    setMotorCommand(motorID, pwmCommand, out);
  }
  return out;
}

const MainLoopOutput userSetDesiredPWMCommand(const MainLoopInput& in,
                                              const MotorID motorID) {
  MainLoopOutput out;
  // only set the value of the blue button is pressed
  if (in.joystickInput.buttonBlue) {
    // select the desired pwm signal
    const int pwmCommand =
        UserInputState::desiredPWM[UserInputState::currentPWMIndex];
    // set the desired speed for the motors as specified
    setMotorCommand(motorID, pwmCommand, out);
  }
  return out;
}

MainLoopOutput MainLoop(MainLoopInput const &in) {
  // Declaring the output
  MainLoopOutput out;
  // process user input to update states
  updateInputState(in);
  // set the pwm values
  // out = userSetDesiredPWMCommand(in, MotorID::FRONT_LEFT);
  // set speed values
  out = userSetDesiredSpeed(in, MotorID::ALL);
  //out = userSetDesiredPWM(in, MotorID::ALL);

  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = out;
  return out;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("\n");  //new line
  printf("Gyro: ");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("\n");  //new line

  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //printf("Current PWM command: %d\n", desiredPWM[currentPWMIndex]);
  printf("Current Speed command: %d\n", UserInputState::desiredSpeed[UserInputState::currentSpeedIndex]);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  //printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart)
    printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect)
    printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor command 1 = %6.3f\n",
         double(lastMainLoopOutputs.motorCommand1));
}
