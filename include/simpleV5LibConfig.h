#ifndef SIMPLEV5LIBCONFIG_H
#define SIMPLEV5LIBCONFIG_H
#include "vex.h"

using namespace vex;

const int PORT_LEFTFRONT = PORT1; // left front
const int PORT_LEFTMIDDLE = PORT2; // left middle, or left back, depends on your drive config
const int PORT_LEFTBACK = PORT3; // left back, or left high, depends on your drive config
const int PORT_RIGHTFRONT = PORT4;
const int PORT_RIGHTMIDDLE = PORT5;
const int PORT_RIGHTBACK = PORT6;
const int PORT_INERTIAL = PORT7;

const gearSetting LF_GEAR_RATIO = ratio18_1; // half motor
const gearSetting LM_GEAR_RATIO = ratio6_1; // full motor
const gearSetting LB_GEAR_RATIO = ratio6_1; // full motor
const gearSetting RF_GEAR_RATIO = ratio18_1; // half motor
const gearSetting RM_GEAR_RATIO = ratio6_1; // full motor
const gearSetting RB_GEAR_RATIO = ratio6_1; // full motor

 // true = reversed, false = not reversed
const bool LF_DIRECTION = true;
const bool LM_DIRECTION = true;
const bool LB_DIRECTION = false;
const bool RF_DIRECTION = false;
const bool RM_DIRECTION = false;
const bool RB_DIRECTION = true;

// ---------- Turning (PID_turn, PID_turn_relative, PID_turn_shortest) ----------
// Tune PID turn constants
const double TURN_KP = 3.2;
const double TURN_KI = 0.2;
const double TURN_KD = 36.7;

// Turn integral range
const double TURN_INTEGRAL_RANGE = 10;

// Smallest power (percent) used while turning, so friction can't stop the robot just short of the target
const double TURN_MIN_SPEED = 10;

// ---------- Swing turns (PID_swing: only one side of the drivetrain moves) ----------
// Start from the turn values. Swings often need a bigger kP, because only one side pushes.
const double SWING_KP = TURN_KP;
const double SWING_KI = TURN_KI;
const double SWING_KD = TURN_KD;
const double SWING_INTEGRAL_RANGE = TURN_INTEGRAL_RANGE;

// ---------- Driving forward (PID_forward) ----------
// Tune PID forward constants
const double FORWARD_KP = 12;
const double FORWARD_KI = 0.05;
const double FORWARD_KD = 0.78; // re-tune this: the D term now brakes instead of pushing (sign fix)

// Forward integral range
const double FORWARD_INTEGRAL_RANGE = 1.5;

// Keeps the robot driving straight during PID_forward.
// Power added per degree of heading drift. Set to 0 to turn it off.
const double FORWARD_HEADING_KP = 1.0;

// ---------- When is a movement finished? ----------
// Default time limits in milliseconds. If a movement takes longer than this
// (for example the robot is stuck on a wall), it gives up so autonomous can continue.
const double TURN_TIMEOUT_MS = 3000;
const double FORWARD_TIMEOUT_MS = 5000;

// The robot must stay inside the error and speed tolerances for this long (milliseconds)
// before a movement counts as finished. Stops it from "finishing" while flying past the target.
// Set to 0 to finish the moment it is inside the tolerances.
const double TURN_SETTLE_MS = 50;
const double FORWARD_SETTLE_MS = 50;

// What the motors do when a movement ends:
// brakeType::coast (roll freely), brakeType::brake (stop quickly), brakeType::hold (stop and push back)
const brakeType DRIVE_BRAKE_MODE = brakeType::brake;

// ---------- Driver control (tankDrive, arcadeDrive) ----------
// Joystick values smaller than this (percent) are ignored, so a stick that doesn't sit exactly at 0 won't creep
const double DRIVE_DEADBAND = 5;
// 1 = power follows the stick exactly. Bigger numbers give finer control near the middle of the stick,
// while full stick is still full power. 2 means half stick gives a quarter power.
const double DRIVE_CURVE = 2.0;
// arcadeDrive only: turning power is multiplied by this. Lower it (e.g. 0.7) if turning feels too twitchy.
const double DRIVE_TURN_SCALE = 1.0;

// ---------- Robot size ----------
const double WHEEL_DIAMETER_INCH = 3.25;
const double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * M_PI;
const double MOTOR_TO_WHEEL_GEAR_RATIO = 2.0 / 3.0;
#endif
