#ifndef SIMPLEV5LIB_H
#define SIMPLEV5LIB_H
#include "simpleV5LibConfig.h"

using namespace vex;

// ============================================================================
// Devices (created in simpleV5lib.cpp, ports are set in simpleV5LibConfig.h)
// ============================================================================

extern brain Brain;
extern controller Controller;
extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;

// All motors of one side together, so one command drives the whole side
extern motor_group leftDrive;
extern motor_group rightDrive;

extern inertial Inertial;

// ============================================================================
// PID controller
// You can use this for your own mechanisms too (a lift, an arm, ...).
// ============================================================================

struct PIDController {
    double kp;
    double ki;
    double kd;
    double integral_range; // only add up the error when closer to the target than this

    double error_sum = 0;
    double past_error = 0;

    // The three parts of the last compute() result, handy for graphing (see logToTerminal)
    double last_p = 0;
    double last_i = 0;
    double last_d = 0;

    PIDController(double kp, double ki, double kd, double integral_range);

    // Call before starting a new movement
    void reset(double starting_error);

    // error: target - current position
    // rate:  how fast the position is changing right now (positive = position getting bigger)
    // Returns the power to use.
    double compute(double error, double rate);
};

// ============================================================================
// Setup
// ============================================================================

// Call once before autonomous (for example in pre_auton). Takes about 2 seconds.
// Keep the robot still while it runs. Afterwards odometry starts at (0, 0) facing 0.
void calibrateInertial();

// Tell the robot which way it is facing right now, in degrees.
// Useful at the start of an autonomous if the robot doesn't start facing 0.
void setHeading(double degrees);

// ============================================================================
// Odometry: where is the robot on the field?
// Positions are in inches: x to the right, y forward (the way heading 0 points).
// It runs in the background and needs no extra code; tracking wheels are optional
// (see simpleV5LibConfig.h).
// ============================================================================

// Tell the robot where it is right now, for example at the start of an autonomous:
// setPose(0, 0, 0) = "here is (0, 0) and I'm facing 0 degrees"
void setPose(double x, double y, double heading);

double getX(); // inches
double getY(); // inches
// (the heading is getInertial())

// Starts the odometry background task. calibrateInertial and setPose already call it.
void startOdometry();

// ============================================================================
// Sensors
// ============================================================================

double getInertial();  // heading in degrees, clockwise is positive, keeps counting past 360
double getGyroRate();  // turning speed in degrees/10 ms, clockwise is positive
double getPosition();  // distance driven in inches
double getMotorRate(); // driving speed in inches/s

// ============================================================================
// Basic driving
// ============================================================================

// left_speed and right_speed are percentages from -100 to 100
void move(double left_speed, double right_speed);

// Stop all drive motors. Uses DRIVE_BRAKE_MODE unless you pass a different one.
void stopDriving(brakeType mode = DRIVE_BRAKE_MODE);

// ============================================================================
// Autonomous movements
// Every movement waits until it is finished, then stops the drivetrain.
// Each one also has an _async version, see "Doing other things while driving" below.
//   error_tolerance: how close to the target counts as "there"
//   speed_tolerance: how slow the robot must be moving to count as stopped
//   timeout_ms:      (optional) give up after this many milliseconds
//   max_speed:       (optional) highest power to use, in percent (0 to 100)
// ============================================================================

// Drive straight. target in inches, negative drives backwards. Tolerances in inches and inches/s.
void PID_forward(double target, double error_tolerance, double speed_tolerance,
                 double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);

// Turn in place to an exact heading in degrees (see getInertial).
// PID_turn(450) after PID_turn(0) turns 450 degrees, it does not take a shortcut.
// Tolerances in degrees and degrees/10 ms.
void PID_turn(double target, double error_tolerance, double speed_tolerance,
              double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place by some degrees from where the robot is facing now. Positive = clockwise.
void PID_turn_relative(double degrees, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place to face a compass heading (0 to 360), taking the shorter way around.
void PID_turn_shortest(double heading, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place to face the field point (x, y), the shorter way around.
// backwards = true points the back of the robot at it. Tolerances in degrees and degrees/10 ms.
void PID_turn_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Drive to the field point (x, y), using odometry. Turns towards it first if needed, and keeps
// aiming at it while driving. backwards = true drives there in reverse.
// Tolerances in inches and inches/s.
void PID_drive_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                        double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Which side of the drivetrain moves during a swing turn
enum driveSide { LEFT_SIDE, RIGHT_SIDE };

// Swing turn to an exact heading: only moving_side drives, the other side holds still.
// Tolerances in degrees and degrees/10 ms.
void PID_swing(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
               double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Drive along a circle until the robot faces target (degrees, like PID_turn).
// radius: inches from the middle of the circle to the middle of the robot. Positive drives forward
// along the arc, negative drives backwards. Bigger radius = gentler curve, 0 = turn in place.
// Tolerances in degrees and degrees/10 ms. Needs TRACK_WIDTH_INCH in simpleV5LibConfig.h.
void PID_arc(double target, double radius, double error_tolerance, double speed_tolerance,
             double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);

// ============================================================================
// Doing other things while driving
// The _async versions start the movement and return right away, so your code can
// run an intake, raise a lift, ... while the robot drives. Same parameters as above.
// A new movement always waits for the previous one to finish first.
//
//   PID_forward_async(36, 0.3, 0.2);
//   waitUntilTraveled(12);       // after 12 inches...
//   intake.spin(forward);        // ...start the intake
//   waitUntilDone();             // wait for the drive to finish
// ============================================================================

void PID_forward_async(double target, double error_tolerance, double speed_tolerance,
                       double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_async(double target, double error_tolerance, double speed_tolerance,
                    double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_turn_relative_async(double degrees, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_turn_shortest_async(double heading, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_swing_async(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
                     double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_arc_async(double target, double radius, double error_tolerance, double speed_tolerance,
                   double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100, bool backwards = false);
void PID_drive_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                              double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Wait until the current movement is finished
void waitUntilDone();

// Wait until the current movement has gone this far from where it started
// (inches for PID_forward and PID_drive_to_point, degrees for turns), or has finished
void waitUntilTraveled(double amount);

// true while a movement is running
bool isMoving();

// Stop the current movement right away, or a chained one that is still rolling
// (also called by tankDrive / arcadeDrive)
void cancelMovement();

// ============================================================================
// Chained movements: don't stop between movements
// A chained movement finishes when it gets within exit_range of its target (inches for
// PID_forward_chain, degrees for the others) and does NOT stop: the next movement takes
// over while the robot is still moving. Much faster, a little less exact.
// End your chain with a normal movement so the robot stops at the right place.
// (If nothing follows within CHAIN_STOP_AFTER_MS, the drivetrain stops on its own.)
//
//   PID_forward_chain(24, 3);          // drive 24 inches, move on 3 inches before the end
//   PID_turn_chain(90, 10);            // turn to 90, move on 10 degrees before
//   PID_forward(24, 0.3, 0.2);         // normal movement: stops at the end
// ============================================================================

void PID_forward_chain(double target, double exit_range, double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_chain(double target, double exit_range, double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_swing_chain(double target, driveSide moving_side, double exit_range,
                     double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_arc_chain(double target, double radius, double exit_range,
                   double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);

// ============================================================================
// Live data, for tuning
// Shows what the PID is doing during every movement. Both are off by default.
// ============================================================================

// Print one line every 20 ms to the terminal (connect with a USB cable):
//   time_ms,error,speed,output,p,i,d
// Copy it into a spreadsheet and make a line chart to see the movement.
void logToTerminal(bool on);

// Draw the error (red) and the power (green) on the Brain screen while the robot moves.
// The middle line is 0: a red line that crosses it means the robot went past the target.
void graphOnScreen(bool on);

// Used by the movements to report their data
void telemetryStart(const char* name, double target, double start_error, double timeout_ms);
void telemetryUpdate(double time_ms, double error, double speed, double output, const PIDController &pid);

// ============================================================================
// Driver control: call one of these inside the while loop in usercontrol()
// ============================================================================

// Left stick up/down drives the left side, right stick up/down drives the right side
void tankDrive();

// Left stick up/down drives forward and back, right stick left/right turns
void arcadeDrive();

// ============================================================================
// Autonomous selector
// Pick which autonomous to run from the Brain screen (tap left / right half)
// or the controller (Left / Right arrow buttons).
// ============================================================================

// Add a routine to the list, with a short name (up to 10 routines)
void addAuton(const char* name, void (*routine)());

// Call at the end of pre_auton, after addAuton
void startAutonSelector();

// Call at the start of usercontrol, so the arrow buttons are free for driving
void stopAutonSelector();

// Call in autonomous(): runs the routine that is selected
void runSelectedAuton();

#endif
