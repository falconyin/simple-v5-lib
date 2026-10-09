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

    last_p = error * kp;
    last_i = error_sum * ki;
    // D: brake. Moving towards a bigger position makes the error smaller, so push against the speed
    last_d = -rate * kd;
    return last_p + last_i + last_d;
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
//
// Every movement runs in one background task (motionLoop). The normal functions
// (PID_forward, ...) start the movement and then wait for it; the _async ones
// start it and return right away, so your code can do other things meanwhile.
// ============================================================================

enum motionType { MOTION_FORWARD, MOTION_TURN, MOTION_SWING_LEFT, MOTION_SWING_RIGHT };

struct MotionRequest {
    motionType type;
    double target;
    double error_tolerance;
    double speed_tolerance;
    double timeout_ms;
    double max_speed;
};

// Shared between your code and the background task ("volatile" = may change at any time)
static volatile bool motion_running = false;   // a movement is started and not finished yet
static volatile bool motion_requested = false; // a new movement is waiting for the task to pick it up
static volatile bool cancel_requested = false; // cancelMovement() asked the movement to stop
static volatile double motion_progress = 0;    // how far the current movement has gone (inches or degrees)
static MotionRequest next_motion;
static task* motion_task = nullptr;

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
static void turnToHeading(const MotionRequest &request, PIDController pid, turnStyle style, const char* name) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    SettleCheck settle(TURN_SETTLE_MS);
    double start_heading = getInertial();
    double target = request.target;
    pid.reset(target - start_heading);
    telemetryStart(name, target, target - start_heading, request.timeout_ms);

    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double current_heading = getInertial();
        double current_error = target - current_heading;
        double gyro_rate = getGyroRate();
        motion_progress = fabs(current_heading - start_heading);
        bool inside = fabs(current_error) < request.error_tolerance && fabs(gyro_rate) < request.speed_tolerance;
        if (settle.update(inside, time)) {
            break;
        }

        double total_correction = cap(pid.compute(current_error, gyro_rate), request.max_speed);

        // Almost stopped but not there yet: give it a minimum push to beat friction
        double min_speed = fmin(TURN_MIN_SPEED, request.max_speed);
        if (fabs(current_error) > request.error_tolerance && fabs(total_correction) < min_speed
            && fabs(gyro_rate) < request.speed_tolerance) {
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
        telemetryUpdate(time, current_error, gyro_rate, total_correction, pid);
        vexDelay(delay);
    }
    stopDriving();
}

static void driveForward(const MotionRequest &request) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    double target = request.target;
    double start_position = getPosition();
    double start_heading = getInertial(); // the heading we try to keep while driving
    pid.reset(target);
    telemetryStart("PID_forward", target, target, request.timeout_ms);

    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double driven = getPosition() - start_position;
        double current_error = target - driven;
        double motor_rate = getMotorRate();
        motion_progress = fabs(driven);
        bool inside = fabs(current_error) < request.error_tolerance && fabs(motor_rate) < request.speed_tolerance;
        if (settle.update(inside, time)) {
            break;
        }

        double total_correction = cap(pid.compute(current_error, motor_rate), request.max_speed);

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
        telemetryUpdate(time, current_error, motor_rate, total_correction, pid);
        vexDelay(delay);
    }
    stopDriving();
}

static void runMotion(const MotionRequest &request) {
    if (request.type == MOTION_FORWARD) {
        driveForward(request);
    } else if (request.type == MOTION_TURN) {
        PIDController pid(TURN_KP, TURN_KI, TURN_KD, TURN_INTEGRAL_RANGE);
        turnToHeading(request, pid, POINT_TURN, "PID_turn");
    } else {
        PIDController pid(SWING_KP, SWING_KI, SWING_KD, SWING_INTEGRAL_RANGE);
        turnStyle style = (request.type == MOTION_SWING_LEFT) ? LEFT_SWING : RIGHT_SWING;
        turnToHeading(request, pid, style, "PID_swing");
    }
}

// The background task: waits for a movement, runs it, repeats
static int motionLoop() {
    while (true) {
        if (motion_requested) {
            motion_requested = false;
            runMotion(next_motion);
            motion_running = false;
        }
        vexDelay(5);
    }
    return 0;
}

// Hand a movement to the background task. Waits for the previous movement to finish first.
static void startMotion(motionType type, double target, double error_tolerance, double speed_tolerance,
                        double timeout_ms, double max_speed) {
    waitUntilDone();
    if (motion_task == nullptr) {
        // Made with "new" so the task object is never destroyed and the task keeps running
        motion_task = new task(motionLoop);
    }
    next_motion = {type, target, error_tolerance, speed_tolerance, timeout_ms, max_speed};
    cancel_requested = false;
    motion_progress = 0;
    motion_running = true;
    motion_requested = true;
}

// Turn targets for relative and shortest-way turns, worked out from where the robot faces now
static double relativeTarget(double degrees) {
    return getInertial() + degrees;
}

static double shortestTarget(double heading) {
    double current = getInertial();
    // How far to turn, squeezed into -180 to 180 so we always go the short way
    double difference = fmod(heading - current, 360);
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }
    return current + difference;
}

static motionType swingType(driveSide moving_side) {
    return (moving_side == LEFT_SIDE) ? MOTION_SWING_LEFT : MOTION_SWING_RIGHT;
}

// ---------- Start a movement and return right away ----------

void PID_forward_async(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    startMotion(MOTION_FORWARD, target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_turn_async(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    startMotion(MOTION_TURN, target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_turn_relative_async(double degrees, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    waitUntilDone(); // measure "from where the robot is facing" after the previous movement ends
    PID_turn_async(relativeTarget(degrees), error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_turn_shortest_async(double heading, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    waitUntilDone();
    PID_turn_async(shortestTarget(heading), error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_swing_async(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
                     double timeout_ms, double max_speed) {
    startMotion(swingType(moving_side), target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

// ---------- Start a movement and wait until it is finished ----------

void PID_forward(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PID_forward_async(target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

void PID_turn(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PID_turn_async(target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

void PID_turn_relative(double degrees, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PID_turn_relative_async(degrees, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

void PID_turn_shortest(double heading, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    PID_turn_shortest_async(heading, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

void PID_swing(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
               double timeout_ms, double max_speed) {
    PID_swing_async(target, moving_side, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

// ---------- Waiting and stopping ----------

void waitUntilDone() {
    while (motion_running) {
        vexDelay(5);
    }
}

void waitUntilTraveled(double amount) {
    while (motion_running && motion_progress < amount) {
        vexDelay(5);
    }
}

bool isMoving() {
    return motion_running;
}

void cancelMovement() {
    if (!motion_running) {
        return;
    }
    cancel_requested = true;
    waitUntilDone();
    cancel_requested = false;
}
