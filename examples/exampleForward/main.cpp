/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ianyin                                                    */
/*    Created:      6/28/2026, 16:09:01 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

void pre_auton(void) {
}

void autonomous(void) {
}

void usercontrol(void) {
}

int main() {
    calibrateInertial(); // keep the robot still for about 2 seconds while the inertial sensor calibrates

    PID_forward(23.622, 0.3, 0.2);
    // The first parameter is target distance, in inches. Use a negative number to drive backwards.
    // The second parameter is the error tolerance, in inches. This is the value in which if the error (distance to the target), is within, the movement will end
    // The third parameter is the speed tolerance, in inch/s. Again, same logic as the error tolerance.
    // The fourth parameter (optional) is the timeout, in milliseconds. If the movement takes longer than this, it stops. Default: FORWARD_TIMEOUT_MS
    // While driving, the robot keeps the heading it had at the start (see FORWARD_HEADING_KP).
}