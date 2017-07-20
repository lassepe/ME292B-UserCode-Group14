#include "UserCode.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep these around for later debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

MainLoopOutput MainLoop(MainLoopInput const &in) {
  lastMainLoopInputs = in;//we copy this, in case we want to print some info about the inputs
  //Your code goes here!

  //Define the output numbers:
  MainLoopOutput outVals;
  outVals.motorCommand1 = 0;
  outVals.motorCommand2 = 0;
  outVals.motorCommand3 = 0;
  outVals.motorCommand4 = 0;

  outVals.led1 = 0;
  outVals.led2 = 0;
  outVals.led3 = 0;
  outVals.led4 = 0;

  //copy the outputs:
  lastMainLoopOutputs = outVals;
  return outVals;
}

void PrintStatus() {
  printf("\n\n---- Begin print status ----\n");
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.

  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n", double(lastMainLoopInputs.batteryVoltage.value));
  printf("Last main loop outputs:\n");
  printf("  motor command 1 = %6.3f\n", double(lastMainLoopOutputs.motorCommand1));
  printf("==== End print status ====\n");
}
