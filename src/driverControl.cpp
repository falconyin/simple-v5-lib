#include "simpleV5lib.h"

// Turns a joystick value (-100 to 100) into a drive power (-100 to 100):
// small values (the deadband) become 0, and DRIVE_CURVE gives finer control near the middle.
static double stickToPower(double stick) {
    if (fabs(stick) < DRIVE_DEADBAND) {
        return 0;
    }
    double sign = (stick >= 0) ? 1 : -1;
    return sign * pow(fabs(stick) / 100, DRIVE_CURVE) * 100;
}

// Driver control and an autonomous movement would fight over the motors, so the driver wins
static void stopAutonomousMovement() {
    cancelMovement(); // does nothing if no movement is running
}

void tankDrive() {
    stopAutonomousMovement();
    double left = stickToPower(Controller.Axis3.position(percentUnits::pct));  // left stick up/down
    double right = stickToPower(Controller.Axis2.position(percentUnits::pct)); // right stick up/down
    move(left, right);
}

void arcadeDrive() {
    stopAutonomousMovement();
    double forward = stickToPower(Controller.Axis3.position(percentUnits::pct));                // left stick up/down
    double turn = stickToPower(Controller.Axis1.position(percentUnits::pct)) * DRIVE_TURN_SCALE; // right stick left/right
    move(forward + turn, forward - turn);
}
