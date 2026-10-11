/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  An arm that holds its position and an intake that frees  */
/*                  itself when it jams, in autonomous and driver control     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

competition Competition;

// ---------- Mechanisms ----------
// Change the ports to yours. Made here, outside any function, so they exist the whole program.
motor armMotor(PORT9, ratio36_1, false);
Arm arm(armMotor);

// Two motors that always move together? Put them in a motor_group:
//   motor armLeft(PORT9, ratio36_1, false);
//   motor armRight(PORT10, ratio36_1, true);
//   motor_group armMotors(armLeft, armRight);
//   Arm arm(armMotors);

motor intakeMotor(PORT11, ratio6_1, false);
Intake intake(intakeMotor);

// Arm positions, in degrees of the motor. Find yours: move the arm by hand (or with the stick)
// and watch the number on the Brain screen (see usercontrol).
const double ARM_DOWN = 0;
const double ARM_SCORE = 450;
const double ARM_HIGHEST = 600;

// ---------- Autonomous ----------

void autonomous(void) {
    setHeading(0);
    intake.spin(100);                // runs in the background, frees itself if a piece gets stuck
    PID_forward(24, 0.3, 0.2);       // drive while the intake runs
    arm.moveTo(ARM_SCORE);           // starts raising the arm...
    PID_turn(90, 0.5, 0.2);          // ...while the robot turns
    arm.waitUntilDone();             // make sure it's up before scoring
    intake.spin(-100);               // push the piece out
    wait(500, msec);
    intake.stop();
    arm.moveTo(ARM_DOWN);            // lower it again, no need to wait

    // Intake until a game piece is all the way in: with unjam = false it stops when the piece hits
    // the end and the motor can't turn any more
    intake.spin(100, false);
    PID_forward(12, 0.3, 0.2);
    double start = Brain.timer(msec);
    while (!intake.isJammed() && Brain.timer(msec) - start < 2000) {
        wait(10, msec);
    }
    intake.stop();
}

// ---------- Driver control ----------

void usercontrol(void) {
    cancelMovement();
    while (true) {
        arcadeDrive();

        // Arm: right stick up/down moves it, let go and it holds. A button sends it to a position.
        if (Controller.ButtonL1.pressing()) {
            arm.moveTo(ARM_SCORE);
        } else if (Controller.ButtonL2.pressing()) {
            arm.moveTo(ARM_DOWN);
        }
        arm.manual(Controller.Axis2.position(percentUnits::pct));

        // Intake: R1 in, R2 out. Calling spin with the same power every loop is fine.
        if (Controller.ButtonR1.pressing()) {
            intake.spin(100);
        } else if (Controller.ButtonR2.pressing()) {
            intake.spin(-100);
        } else {
            intake.stop();
        }

        Brain.Screen.printAt(10, 40, true, "Arm: %.0f degrees   ", arm.position());
        Brain.Screen.printAt(10, 60, true, "Intake jams: %d   ", intake.jamCount());
        wait(20, msec);
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    calibrateInertial();
    arm.resetPosition(ARM_DOWN);            // the arm rests all the way down at the start
    arm.setLimits(ARM_DOWN, ARM_HIGHEST);   // the stick can't drive it past these

    while (true) {
        wait(100, msec);
    }
}
