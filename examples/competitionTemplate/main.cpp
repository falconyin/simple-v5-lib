/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  A full competition program using simple-v5-lib:           */
/*                  autonomous selector, autonomous routines, driver control  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

competition Competition;

// ---------- Autonomous routines ----------
// These are just examples, replace them with your own routes.

void leftSideAuton() {
    setHeading(0);                    // the robot starts facing 0 degrees
    PID_forward(23.622, 0.3, 0.2);    // drive one tile forward
    PID_turn(90, 0.5, 0.2);           // turn to face 90 degrees (clockwise)
    PID_forward(-12, 0.3, 0.2);       // back up 12 inches
}

void rightSideAuton() {
    setHeading(0);
    PID_forward(23.622, 0.3, 0.2);
    PID_swing(-45, RIGHT_SIDE, 0.5, 0.2);              // only the right side moves, the left side holds still
    PID_forward(10, 0.3, 0.2, FORWARD_TIMEOUT_MS, 50); // slowly: at most 50% power
}

void skillsAuton() {
    setHeading(0);
    // Do something else while driving: start the drive, and keep going in this code
    PID_forward_async(48, 0.3, 0.2);
    waitUntilTraveled(24);            // wait until the robot has driven 24 inches
    // intake.spin(forward);          // ...then start your intake (or anything else)
    waitUntilDone();                  // wait until the drive is finished

    PID_turn_relative(180, 0.5, 0.2); // turn around, whichever way the robot is facing
    PID_turn_shortest(0, 0.5, 0.2);   // face 0 degrees again, taking the shorter way
}

// ---------- Competition parts ----------

void pre_auton(void) {
    calibrateInertial(); // keep the robot still for about 2 seconds

    addAuton("Left side", leftSideAuton);
    addAuton("Right side", rightSideAuton);
    addAuton("Skills", skillsAuton);
    startAutonSelector(); // tap the Brain screen or use the Left / Right buttons to choose
}

void autonomous(void) {
    runSelectedAuton();
}

void usercontrol(void) {
    stopAutonSelector(); // frees the Left / Right buttons
    cancelMovement();    // stop any autonomous movement that is still running
    while (true) {
        arcadeDrive();   // or tankDrive();
        wait(20, msec);  // don't hog the CPU
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}
