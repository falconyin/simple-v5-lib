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

    PID_turn(90, 0.3, 0.2);
    // The first parameter is target angle, in degrees. Note that this is absolute value, meaning that its axis is in accordance to the field
    // The second parameter is error tolerance, in deg.
    // The third parameter is speed tolerance, in deg/10ms.
    // The fourth parameter (optional) is the timeout, in milliseconds. If the turn takes longer than this, it stops. Default: TURN_TIMEOUT_MS
}