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
// Keep the robot still while it runs.
void calibrateInertial();

// Tell the robot which way it is facing right now, in degrees.
// Useful at the start of an autonomous if the robot doesn't start facing 0.
void setHeading(double degrees);

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

// Which side of the drivetrain moves during a swing turn
enum driveSide { LEFT_SIDE, RIGHT_SIDE };

// Swing turn to an exact heading: only moving_side drives, the other side holds still.
// Tolerances in degrees and degrees/10 ms.
void PID_swing(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
               double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

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
