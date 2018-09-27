#pragma once

#include "MainLoopTypes.hpp"
#include "UtilityFunctions.hpp"

/* Main loop function
 *
 * This is where the action happens. This is run at 500Hz, i.e. every 2ms.
 * The input is a list of sensor measurements, etc., and the outputs are the
 * desired motor commands and the states of the LEDs.
 */
MainLoopOutput MainLoop(MainLoopInput const &in);
/**
 * @brief userSetDesiredPWMCommand lets the user pick from a set of PWM signals
 * and cycle through them using the yellow and green button.
 * @param in the input of the user
 * @param motorID the id of the motor that we would like to control
 * @return the motor commands as selected by the user
 */
const MainLoopOutput userSetDesiredPWMCommand(
    const MainLoopInput& in, const MotorID motorID = MotorID::FRONT_LEFT);
/**
 * @brief userSetDesiredSpeed lets the user pick from a set of speeds and
 * cycle through them using the yellow and green button.
 * @param in the input of the user
 * @return the motor commands as MainLoopOutput that should reach the desired
 * speed selected by the user.
 */
const MainLoopOutput userSetDesiredSpeed(const MainLoopInput& in,
                                         const MotorID motorID = MotorID::ALL);
/**
 *
 * @brief updateInputState updates the state variables of the user inut (e.g.
 * for debouncing)
 * @param in the input of the user
 */
void updateInputState(const MainLoopInput& in);
/* A debugging function, that we can call from the shell, to print information
 * about the system state.
 */
void PrintStatus();
