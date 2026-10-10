/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  Odometry: drive to points on the field                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

int main() {
    calibrateInertial(); // keep the robot still for about 2 seconds
    setPose(0, 0, 0);    // "the robot is at (0, 0), facing 0 degrees": x is to the right, y is forward

    // Drive a square, 24 inches per side, using field points instead of distances and angles
    PID_drive_to_point(0, 24, 0.5, 0.2);   // 24 inches forward
    PID_drive_to_point(24, 24, 0.5, 0.2);  // turns right by itself, then drives
    PID_drive_to_point(24, 0, 0.5, 0.2);
    PID_drive_to_point(0, 0, 0.5, 0.2);    // back to the start
    PID_turn_to_point(0, 24, 0.5, 0.2);    // face the first corner again

    // Back up to (0, -12) without turning around: backwards = true
    PID_drive_to_point(0, -12, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, true);

    // The square's corners again, as one smooth path (starting from (0, -12), where the robot is now,
    // so the first side is 36 inches): chained points don't stop, they curve from one point to the
    // next. The last point is a normal movement, so the robot stops exactly there.
    PID_drive_to_point_chain(0, 24, 4);    // move on 4 inches before the point
    PID_drive_to_point_chain(24, 24, 4);
    PID_drive_to_point_chain(24, 0, 4);
    PID_drive_to_point(0, 0, 0.5, 0.2);

    // Show where the robot thinks it is. Push it around by hand and watch the numbers change.
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 40, true, "x %.1f  y %.1f  heading %.1f", getX(), getY(), getInertial());
        wait(100, msec);
    }
}
