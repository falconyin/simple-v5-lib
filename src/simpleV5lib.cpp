#include "simpleV5lib.h"

// ============================================================================
// Devices
// ============================================================================

brain Brain;
controller Controller;
motor leftFront(PORT_LEFTFRONT, LF_GEAR_RATIO, LF_DIRECTION);
motor leftMiddle(PORT_LEFTMIDDLE, LM_GEAR_RATIO, LM_DIRECTION);
motor leftBack(PORT_LEFTBACK, LB_GEAR_RATIO, LB_DIRECTION);
motor rightFront(PORT_RIGHTFRONT, RF_GEAR_RATIO, RF_DIRECTION);
motor rightMiddle(PORT_RIGHTMIDDLE, RM_GEAR_RATIO, RM_DIRECTION);
motor rightBack(PORT_RIGHTBACK, RB_GEAR_RATIO, RB_DIRECTION);

motor_group leftDrive(leftFront, leftMiddle, leftBack);
motor_group rightDrive(rightFront, rightMiddle, rightBack);

inertial Inertial(PORT_INERTIAL);

// ============================================================================
// Small helpers
// ============================================================================

static double cap(double input, double max_value) {
    if (fabs(input) > max_value) {
        return (input >= 0) ? max_value : max_value * -1.0;
    }
    return input;
}

static double getSign(double input) {
    if (input >= 0) {
        return 1;
    } else {
        return -1;
    }
}

// ============================================================================
// PID controller
// ============================================================================

PIDController::PIDController(double kp, double ki, double kd, double integral_range)
    : kp(kp), ki(ki), kd(kd), integral_range(integral_range) {}

void PIDController::reset(double starting_error) {
    error_sum = 0;
    past_error = starting_error;
}

double PIDController::compute(double error, double rate) {
    // I: add up the error, but only near the target, so it can't build up a huge push on the way there
    if (fabs(error) < integral_range) {
        error_sum = error_sum + error;
    } else {
        error_sum = 0;
    }
    // We just went past the target: throw away the old push, or it would drive us further past
    if ((error * past_error) < 0) {
        error_sum = 0;
    }
    past_error = error;

    double porportional_correction = error * kp;
    double integral_correction = error_sum * ki;
    // D: brake. Moving towards a bigger position makes the error smaller, so push against the speed
    double derivative_correction = -rate * kd;
    return porportional_correction + integral_correction + derivative_correction;
}

// ============================================================================
// Setup
// ============================================================================

void calibrateInertial() {
    Inertial.calibrate();
    while (Inertial.isCalibrating()) {
        vexDelay(10);
    }
}

void setHeading(double degrees) {
    Inertial.setRotation(degrees, rotationUnits::deg);
    // heading() only goes from 0 to 360, so wrap the value into that range
    double heading = fmod(degrees, 360);
    if (heading < 0) {
        heading += 360;
    }
    Inertial.setHeading(heading, rotationUnits::deg);
}

// ============================================================================
// Sensors
// ============================================================================

double getInertial() {
    return Inertial.rotation(rotationUnits::deg);
}

// Turning speed in degrees/10 ms, positive in the same direction as getInertial() (clockwise).
// The IMU's z-axis gyro rate is positive the other way (counter-clockwise), so we flip it.
// If your turns start oscillating wildly after an update, this sign is the first thing to check.
double getGyroRate() {
    return -Inertial.gyroRate(zaxis, dps) / 100;
}

// Distance driven in inches, averaged over the left and right side
double getPosition() {
    double motor_revs = (leftFront.position(rotationUnits::rev) + rightFront.position(rotationUnits::rev)) / 2;
    return motor_revs * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO;
}

// Driving speed in inches/s, averaged over the left and right side
double getMotorRate() {
    double motor_dps = (leftFront.velocity(velocityUnits::dps) + rightFront.velocity(velocityUnits::dps)) / 2;
    return motor_dps / 360 * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO;
}

// ============================================================================
// Basic driving
// ============================================================================

// Power one side, in percent (-100 to 100). 100% = 12 V = 12000 mV.
static void spinSide(motor_group &side, double speed) {
    side.spin(directionType::fwd, speed * 120, voltageUnits::mV);
}

// If either side asks for more than 100, both sides are scaled down together
// so the ratio between them (which steers the robot) is kept.
void move(double left_speed, double right_speed) {
    double biggest = fmax(fabs(left_speed), fabs(right_speed));
    if (biggest > 100) {
        left_speed = left_speed / biggest * 100;
        right_speed = right_speed / biggest * 100;
    }
    spinSide(leftDrive, left_speed);
    spinSide(rightDrive, right_speed);
}

void stopDriving(brakeType mode) {
    leftDrive.stop(mode);
    rightDrive.stop(mode);
}

// ============================================================================
// Autonomous movements
// ============================================================================

// Keeps track of whether a movement is finished: inside the tolerances for at least settle_ms.
// Call update() once per loop. It returns true when the movement is done.
struct SettleCheck {
    double settle_ms;
    double inside_since = -1; // time we got inside the tolerances, -1 = not inside right now

    SettleCheck(double settle_ms) : settle_ms(settle_ms) {}

    bool update(bool inside_tolerances, double now_ms) {
        if (!inside_tolerances) {
            inside_since = -1;
            return false;
        }
        if (inside_since < 0) {
            inside_since = now_ms;
        }
        return now_ms - inside_since >= settle_ms;
    }
};

// How the drivetrain turns: both sides (in place), or only one side (swing)
enum turnStyle { POINT_TURN, LEFT_SWING, RIGHT_SWING };

// One loop shared by all turns. Only the way the power reaches the wheels is different.
static void turnToHeading(double target, double error_tolerance, double speed_tolerance,
                          double timeout_ms, double max_speed, PIDController pid, turnStyle style) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    SettleCheck settle(TURN_SETTLE_MS);
    pid.reset(target - getInertial());

    while (true) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double current_error = target - getInertial();
        double gyro_rate = getGyroRate();
        bool inside = fabs(current_error) < error_tolerance && fabs(gyro_rate) < speed_tolerance;
        if (settle.update(inside, time)) {
            break;
        }

        double total_correction = cap(pid.compute(current_error, gyro_rate), max_speed);

        // Almost stopped but not there yet: give it a minimum push to beat friction
        double min_speed = fmin(TURN_MIN_SPEED, max_speed);
        if (fabs(current_error) > error_tolerance && fabs(total_correction) < min_speed
            && fabs(gyro_rate) < speed_tolerance) {
            total_correction = getSign(current_error) * min_speed;
        }

        // Positive total_correction turns clockwise
        if (style == POINT_TURN) {
            move(total_correction, total_correction * -1);
        } else if (style == LEFT_SWING) {
            spinSide(leftDrive, total_correction); // left side forward = clockwise
            rightDrive.stop(brakeType::hold);
        } else {
            spinSide(rightDrive, total_correction * -1); // right side backward = clockwise
            leftDrive.stop(brakeType::hold);
        }
        vexDelay(delay);
    }
    stopDriving();
}

void PID_turn(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PIDController pid(TURN_KP, TURN_KI, TURN_KD, TURN_INTEGRAL_RANGE);
    turnToHeading(target, error_tolerance, speed_tolerance, timeout_ms, max_speed, pid, POINT_TURN);
}

void PID_turn_relative(double degrees, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PID_turn(getInertial() + degrees, error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_turn_shortest(double heading, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    double current = getInertial();
    // How far to turn, squeezed into -180 to 180 so we always go the short way
    double difference = fmod(heading - current, 360);
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }
    PID_turn(current + difference, error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_swing(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
               double timeout_ms, double max_speed) {
    PIDController pid(SWING_KP, SWING_KI, SWING_KD, SWING_INTEGRAL_RANGE);
    turnStyle style = (moving_side == LEFT_SIDE) ? LEFT_SWING : RIGHT_SWING;
    turnToHeading(target, error_tolerance, speed_tolerance, timeout_ms, max_speed, pid, style);
}

void PID_forward(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    double start_position = getPosition();
    double start_heading = getInertial(); // the heading we try to keep while driving
    pid.reset(target);

    while (true) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double current_error = target - (getPosition() - start_position);
        double motor_rate = getMotorRate();
        bool inside = fabs(current_error) < error_tolerance && fabs(motor_rate) < speed_tolerance;
        if (settle.update(inside, time)) {
            break;
        }

        double total_correction = cap(pid.compute(current_error, motor_rate), max_speed);

        // Speed up gently during the first 0.3 s (forwards and backwards) so the wheels don't slip
        // (only limits how hard it pushes, the direction still comes from the PID)
        double ramp_limit = 30 + (time / 1000 * 233);
        if (time < 300 && fabs(total_correction) > ramp_limit) {
            total_correction = getSign(total_correction) * ramp_limit;
        }

        // Keep driving straight: if the robot turned clockwise, heading_correction is negative,
        // which slows the left side and speeds up the right side to turn back
        double heading_correction = (start_heading - getInertial()) * FORWARD_HEADING_KP;

        move(total_correction + heading_correction, total_correction - heading_correction);
        vexDelay(delay);
    }
    stopDriving();
}
