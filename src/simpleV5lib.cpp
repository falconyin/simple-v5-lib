#include "simpleV5lib.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>

brain Brain;
controller Controller;
motor leftFront(PORT_LEFTFRONT, LF_GEAR_RATIO, LF_DIRECTION);
motor leftMiddle(PORT_LEFTMIDDLE, LM_GEAR_RATIO, LM_DIRECTION);
motor leftBack(PORT_LEFTBACK, LB_GEAR_RATIO, LB_DIRECTION);
motor rightFront(PORT_RIGHTFRONT, RF_GEAR_RATIO, RF_DIRECTION);
motor rightMiddle(PORT_RIGHTMIDDLE, RM_GEAR_RATIO, RM_DIRECTION);
motor rightBack(PORT_RIGHTBACK, RB_GEAR_RATIO, RB_DIRECTION);

inertial Inertial(PORT_INERTIAL);

void calibrateInertial() {
    Inertial.calibrate();
    while (Inertial.isCalibrating()) {
        vexDelay(10);
    }
}

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

void stopDriving() {
    leftFront.stop(brakeType::brake);
    leftMiddle.stop(brakeType::brake);
    leftBack.stop(brakeType::brake);
    rightFront.stop(brakeType::brake);
    rightMiddle.stop(brakeType::brake);
    rightBack.stop(brakeType::brake);
}

// left_speed and right_speed are percentages from -100 to 100.
// If either side asks for more than 100, both sides are scaled down together
// so the ratio between them (which steers the robot) is kept.
void move(double left_speed, double right_speed) {
    double biggest = fmax(fabs(left_speed), fabs(right_speed));
    if (biggest > 100) {
        left_speed = left_speed / biggest * 100;
        right_speed = right_speed / biggest * 100;
    }
    left_speed *= 120;
    right_speed *= 120;
    leftFront.spin(directionType::fwd, left_speed, voltageUnits::mV);
    leftMiddle.spin(directionType::fwd, left_speed, voltageUnits::mV);
    leftBack.spin(directionType::fwd, left_speed, voltageUnits::mV);
    rightFront.spin(directionType::fwd, right_speed, voltageUnits::mV);
    rightMiddle.spin(directionType::fwd, right_speed, voltageUnits::mV);
    rightBack.spin(directionType::fwd, right_speed, voltageUnits::mV);
}

static double cap(double input, uint32_t max_value) {
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

void PID_turn(double target, double error_tolerance, double speed_tolerance, double timeout_ms) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    double kp = TURN_KP;
    double ki = TURN_KI;
    double kd = TURN_KD;
    double porportional_correction = 0;
    double integral_correction = 0;
    double derivative_correction = 0;
    double current_heading = getInertial();
    double target_heading = target;
    double current_error = target_heading - current_heading;
    double past_error = current_error;
    double error_sum = 0;
    double total_correction = 0;
    double integral_range = TURN_INTEGRAL_RANGE;
    double gyro_rate = getGyroRate();

    while (fabs(current_error) > error_tolerance || fabs(gyro_rate) > speed_tolerance) {
        if (Brain.timer(timeUnits::msec) - start_time > timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }
        current_heading = getInertial();
        current_error = target_heading - current_heading;

        if (fabs(current_error) < integral_range) {
            error_sum = error_sum + current_error;
        } else {
            error_sum = 0;
        }
        if ((current_error * past_error) < 0) {
            error_sum = 0;
        }
        porportional_correction = current_error * kp;
        integral_correction = error_sum * ki;
        // Brake: turning towards a bigger heading makes the error smaller, so push against the turning speed
        derivative_correction = -gyro_rate * kd;
        total_correction = cap(porportional_correction + integral_correction + derivative_correction, 100);

        if (fabs(total_correction) < 10 && fabs(gyro_rate) < speed_tolerance) {
            total_correction = getSign(total_correction) * 10;
        }
        
        move(total_correction, total_correction * -1);
        past_error = current_error;
        vexDelay(delay);
        gyro_rate = getGyroRate(); // fresh reading for the exit check
    }
    move(0, 0);
}

void PID_forward(double target, double error_tolerance, double speed_tolerance, double timeout_ms) {
    double startTime = Brain.timer(timeUnits::sec);
    long delay = 10;
    double kp = FORWARD_KP;
    double ki = FORWARD_KI;
    double kd = FORWARD_KD;

    double porportional_correction = 0;
    double integral_correction = 0;
    double derivative_correction = 0;
    double start_position = getPosition();
    double start_heading = getInertial(); // the heading we try to keep while driving
    double heading_correction = 0;
    double current_position = 0;
    double target_distance = target;
    double current_error = target_distance - current_position;
    double past_error = current_error;
    double error_sum = 0;
    double total_correction = 0;
    double integral_range = FORWARD_INTEGRAL_RANGE;
    double motorRate = getMotorRate();
    double current_time = 0;

    while (fabs(current_error) > error_tolerance || fabs(motorRate) > speed_tolerance) {
        current_time = Brain.timer(timeUnits::sec) - startTime;
        if (current_time * 1000 > timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }
        current_position = getPosition() - start_position;
        current_error = target - current_position;

        if (fabs(current_error) < integral_range) {
            error_sum = error_sum + current_error;
        } else {
            error_sum = 0;
        }
        if ((current_error * past_error) < 0) {
            error_sum = 0;
        }
        porportional_correction = current_error * kp;
        integral_correction = error_sum * ki;
        // Brake: driving towards the target makes the error smaller, so push against the driving speed
        derivative_correction = -motorRate * kd;
        total_correction = cap(porportional_correction + integral_correction + derivative_correction, 100);
        // Speed up gently during the first 0.3 s (forwards and backwards) so the wheels don't slip
        // (only limits how hard it pushes, the direction still comes from the PID)
        double ramp_limit = 30 + (current_time * 233);
        if (current_time < 0.3 && fabs(total_correction) > ramp_limit) {
            total_correction = getSign(total_correction) * ramp_limit;
        }

        // Keep driving straight: if the robot turned clockwise, heading_correction is negative,
        // which slows the left side and speeds up the right side to turn back
        heading_correction = (start_heading - getInertial()) * FORWARD_HEADING_KP;

        move(total_correction + heading_correction, total_correction - heading_correction);
        past_error = current_error;
        vexDelay(delay);
        motorRate = getMotorRate();
    }
    move(0, 0);
}