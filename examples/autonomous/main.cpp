/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  One autonomous routine for both starting spots: written   */
/*                  once in field points, mirrored for the other side.       */
/*                  docs/writing-autonomous.md goes through it step by step.  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

competition Competition;

// ---------- Mechanisms (change the ports to yours, see examples/mechanisms) ----------
motor armMotor(PORT9, ratio36_1, false);
Arm arm(armMotor);
motor intakeMotor(PORT11, ratio6_1, false);
Intake intake(intakeMotor);

const double ARM_DOWN = 0;    // degrees of the arm motor
const double ARM_SCORE = 450;

// ---------- Mirroring ----------
// The two starting spots are mirror images of each other: what is to the right on one side is to
// the left on the other. So the routine is written once, for the left spot, and every x and every
// heading goes through these. Distances (PID_forward) and points straight ahead don't change.
bool mirrored = false; // true = the right starting spot

double flipX(double x) { return mirrored ? -x : x; }                     // a field x
double flipHeading(double heading) { return mirrored ? -heading : heading; } // a heading, or a turn
driveSide flipSide(driveSide side) {                                      // the side of a swing
    if (!mirrored) {
        return side;
    }
    return side == LEFT_SIDE ? RIGHT_SIDE : LEFT_SIDE;
}

// ---------- The routine ----------
// Field points in inches, measured from where the robot starts: x to the right, y forward.
// A game piece lies 30 inches ahead, a goal 24 inches to the right and 48 ahead, facing left
// (the robot scores facing 90), and the robot parks next to its starting spot.
void scoringAuton() {
    double start_time = Brain.timer(msec);
    setPose(0, 0, 0); // the robot starts here, facing 0 (line it up with the tile edge)

    // 1. Drive to the game piece, intake running
    intake.spin(100);
    PID_drive_to_point(flipX(0), 30, 0.5, 0.2);

    // 2. Curve to the goal, arriving facing it, and raise the arm on the way
    arm.moveTo(ARM_SCORE);
    PID_drive_to_pose(flipX(24), 48, flipHeading(90), 0.5, 0.2);
    arm.waitUntilDone(); // the arm must be up before scoring

    // 3. Score: push the game piece out
    intake.spin(-100);
    wait(400, msec);
    intake.stop();

    // 4. Back away from the goal, and lower the arm while driving
    arm.moveTo(ARM_DOWN);
    PID_forward(-10, 0.3, 0.2);

    // 5. Park next to the start: one smooth path, the last point stops exactly
    PID_drive_to_point_chain(flipX(0), 24, 2);
    PID_drive_to_point(flipX(-12), 6, 0.5, 0.2);
    PID_turn_shortest(flipHeading(0), 0.5, 0.2);

    printf("Autonomous took %.1f s\n", (Brain.timer(msec) - start_time) / 1000);
}

void leftSpot() {
    mirrored = false;
    scoringAuton();
}

void rightSpot() {
    mirrored = true;
    scoringAuton();
}

// For a match where your partner's routine needs the room: the robot stays where it is
void doNothing() {}

// ---------- Competition parts ----------

void pre_auton(void) {
    calibrateInertial(); // keep the robot still for about 2 seconds
    arm.resetPosition(ARM_DOWN);
    arm.setLimits(ARM_DOWN, 600);
    addAuton("Left spot", leftSpot);
    addAuton("Right spot", rightSpot);
    addAuton("Do nothing", doNothing);
    startAutonSelector();
}

void autonomous(void) {
    runSelectedAuton();
}

void usercontrol(void) {
    stopAutonSelector();
    cancelMovement(); // stop anything autonomous left running
    intake.stop();
    while (true) {
        arcadeDrive();
        wait(20, msec);
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
