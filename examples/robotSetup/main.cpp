/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  Robot setup: check the wiring and the settings in         */
/*                  simpleV5LibConfig.h, and measure the robot                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Run this once on a new robot (and after rebuilding the drivetrain), before tuning.
// Each step shows what it found on the Brain screen; connect a USB cable to also see it in
// the terminal. Press A on the controller to go to the next step.

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

// Wait until A is pressed and let go
void waitForA() {
    while (!Controller.ButtonA.pressing()) {
        wait(10, msec);
    }
    while (Controller.ButtonA.pressing()) {
        wait(10, msec);
    }
}

int main() {
    // 1. Is everything plugged in, on the ports simpleV5LibConfig.h says? (the robot doesn't move)
    checkDevices();
    waitForA();

    calibrateInertial(); // keep the robot still for about 2 seconds

    // 2. Are the motor directions right, and does the inertial sensor agree?
    //    THE ROBOT TURNS a little each way. Fix anything it finds before going on.
    Controller.Screen.clearScreen();
    Controller.Screen.print("A: test drivetrain");
    waitForA();
    testDrivetrain();
    waitForA();

    // 3. Measure the track width (and the tracking wheel offsets): THE ROBOT SPINS 3 times.
    Controller.Screen.clearScreen();
    Controller.Screen.print("A: measure track");
    waitForA();
    measureTrackWidth();
    waitForA();

    // 4. Measure the wheel size: push the robot by hand, straight forward, 48 inches
    //    (two tiles) along a tape measure. It tells you when to press A.
    measureWheelSize(48);

    // Copy the numbers into simpleV5LibConfig.h, download your program again,
    // and run this once more: the numbers should now hardly change.
}
