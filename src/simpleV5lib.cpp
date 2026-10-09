#include "simpleV5lib.h"
#include <atomic>

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
    startOdometry(); // start keeping track of the position, from (0, 0) facing 0
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

// Like move(), but neither side goes above limit. Both sides are scaled down together,
// so the robot still steers the same way.
static void moveLimited(double left_speed, double right_speed, double limit) {
    double biggest = fmax(fabs(left_speed), fabs(right_speed));
    if (biggest > limit) {
        left_speed = left_speed / biggest * limit;
        right_speed = right_speed / biggest * limit;
    }
    move(left_speed, right_speed);
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

enum motionType { MOTION_FORWARD, MOTION_TURN, MOTION_SWING_LEFT, MOTION_SWING_RIGHT, MOTION_ARC, MOTION_TO_POINT };

struct MotionRequest {
    motionType type;
    double target;
    double error_tolerance;
    double speed_tolerance;
    double timeout_ms;
    double max_speed;
    double radius;     // arcs only: inches, negative = drive backwards along the arc
    double exit_range; // chained movements only: hand over to the next movement this close to the target
    double target_x;   // driving to a point only: the point, in field inches
    double target_y;
    bool backwards;    // driving to a point only: drive there backwards
    bool report_progress; // update motion_progress (for waitUntilTraveled)
};

static MotionRequest makeRequest(motionType type, double target, double error_tolerance, double speed_tolerance,
                                 double timeout_ms, double max_speed);

// Shared between your code and the background task. std::atomic makes sure that when one task
// changes a value, the other task sees the new value (and everything written before it).
static std::atomic<bool> motion_running(false);   // a movement is started and not finished yet
static std::atomic<bool> motion_requested(false); // a new movement is waiting for the task to pick it up
static std::atomic<bool> cancel_requested(false); // cancelMovement() asked the movement to stop
static std::atomic<double> motion_progress(0);    // how far the current movement has gone (inches or degrees)
static MotionRequest next_motion;
static task* motion_task = nullptr;

// Motion chaining: a chained movement ends early on purpose, with the robot still moving
static std::atomic<bool> last_was_chain(false);    // the last movement was chained and nothing has stopped the robot since
static std::atomic<double> chain_heading(0);       // the heading that chained movement was aiming for (or holding)
static std::atomic<double> chain_end_time(-1);     // when it ended, -1 = no chained movement waiting for a follow-up
static std::atomic<bool> chain_was_forward(false); // that chained movement was a PID_forward_chain...
static std::atomic<double> chain_forward_end(0);   // ...and this is where its target was (in getPosition() inches)

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

// Speed up gently during the first 0.3 s so the wheels don't slip
// (only limits how hard it pushes, the direction still comes from the PID)
static double startRamp(double power, double time_ms) {
    double ramp_limit = 30 + (time_ms / 1000 * 233);
    if (time_ms < 300 && fabs(power) > ramp_limit) {
        return getSign(power) * ramp_limit;
    }
    return power;
}

// Called at the end of every movement
static void endMovement(bool chained, double heading_to_keep) {
    if (chained) {
        // Keep the motors running, the next movement takes over without stopping
        last_was_chain = true;
        chain_heading = heading_to_keep;
        chain_end_time = Brain.timer(timeUnits::msec);
    } else {
        last_was_chain = false;
        chain_end_time = -1;
        stopDriving();
    }
}

// Length of a piece of a circle: degrees of turning on a circle with this radius
static double arcLength(double degrees, double radius) {
    return degrees * M_PI / 180 * radius;
}

// An angle squeezed into -180 to 180: the shortest way to turn by it
static double wrap180(double degrees) {
    double wrapped = fmod(degrees, 360);
    if (wrapped > 180) {
        wrapped -= 360;
    } else if (wrapped < -180) {
        wrapped += 360;
    }
    return wrapped;
}

// The heading that faces the field point (x, y) from where the robot is now
// (degrees like getInertial: 0 = straight along +y, clockwise is positive)
static double headingTo(double x, double y) {
    return atan2(x - getX(), y - getY()) * 180 / M_PI;
}

// How far the point (x, y) is in front of the robot, measured along the way it faces
// (negative = the point is behind the robot)
static double distanceAhead(double x, double y, double heading) {
    double facing = heading * M_PI / 180;
    return (x - getX()) * sin(facing) + (y - getY()) * cos(facing);
}

// How the drivetrain turns: both sides (in place), only one side (swing), or along a circle (arc)
enum turnStyle { POINT_TURN, LEFT_SWING, RIGHT_SWING, ARC };

// One loop shared by everything that turns to a heading. Only the way the power reaches the wheels is different.
static void turnToHeading(const MotionRequest &request, PIDController pid, turnStyle style, const char* name) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    SettleCheck settle(TURN_SETTLE_MS);
    bool continuing = last_was_chain; // the robot is still moving from a chained movement
    double start_heading = getInertial();
    double target = request.target;
    double direction = getSign(target - start_heading); // +1 = this turn goes clockwise

    // Chaining: aim a bit past the target, so the robot is still moving when it gets there
    bool chaining = request.exit_range > 0;
    double pid_target = chaining ? target + direction * request.exit_range : target;

    // Arcs: the robot drives along a circle. bend = +1 if driving forward along this circle turns the robot
    // clockwise (the circle bends to the right), -1 if it bends to the left.
    double radius = fabs(request.radius);
    double bend = direction * getSign(request.radius);
    double max_speed = request.max_speed;
    if (style == ARC) {
        // The outside wheels go faster than the middle of the robot, keep them under max_speed
        max_speed = max_speed * radius / (radius + TRACK_WIDTH_INCH / 2);
        pid.reset(arcLength(pid_target - start_heading, radius) * bend);
    } else {
        pid.reset(pid_target - start_heading);
    }
    telemetryStart(name, target, target - start_heading, request.timeout_ms);

    bool chained_exit = false;
    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double current_heading = getInertial();
        double current_error = target - current_heading; // to the real target, not the chaining one
        double gyro_rate = getGyroRate();
        if (request.report_progress) {
            motion_progress = fabs(current_heading - start_heading);
        }

        if (chaining) {
            // Close enough (or already past it): hand over to the next movement without stopping
            if (current_error * direction < request.exit_range) {
                chained_exit = true;
                break;
            }
        } else {
            bool inside = fabs(current_error) < request.error_tolerance && fabs(gyro_rate) < request.speed_tolerance;
            if (settle.update(inside, time)) {
                break;
            }
        }

        double pid_error = pid_target - current_heading;
        double total_correction;
        double push_direction; // which way to push to get closer to the target
        if (style == ARC) {
            // Arcs use the distance left along the circle (positive = forward along it),
            // so the PID works in inches just like PID_forward
            total_correction = pid.compute(arcLength(pid_error, radius) * bend, getMotorRate());
            push_direction = getSign(current_error) * bend;
        } else {
            total_correction = pid.compute(pid_error, gyro_rate);
            push_direction = getSign(current_error);
        }
        total_correction = cap(total_correction, max_speed);
        if (style == ARC && !continuing) {
            total_correction = startRamp(total_correction, time);
        }

        // Almost stopped but not there yet: give it a minimum push to beat friction
        double min_speed = fmin(TURN_MIN_SPEED, max_speed);
        if (!chaining && fabs(current_error) > request.error_tolerance && fabs(total_correction) < min_speed
            && fabs(gyro_rate) < request.speed_tolerance) {
            total_correction = push_direction * min_speed;
        }

        if (style == POINT_TURN) {
            move(total_correction, total_correction * -1); // positive = clockwise
        } else if (style == LEFT_SWING) {
            spinSide(leftDrive, total_correction); // left side forward = clockwise
            rightDrive.stop(brakeType::hold);
        } else if (style == RIGHT_SWING) {
            spinSide(rightDrive, total_correction * -1); // right side backward = clockwise
            leftDrive.stop(brakeType::hold);
        } else {
            // ARC: total_correction is the power for the middle of the robot. The wheels on the
            // outside of the circle have further to go than the ones on the inside.
            double outside = total_correction * (radius + TRACK_WIDTH_INCH / 2) / radius;
            double inside = total_correction * (radius - TRACK_WIDTH_INCH / 2) / radius;
            if (bend > 0) {
                moveLimited(outside, inside, request.max_speed); // circle bends right: the left side is on the outside
            } else {
                moveLimited(inside, outside, request.max_speed);
            }
        }
        telemetryUpdate(time, current_error, gyro_rate, total_correction, pid);
        vexDelay(delay);
    }
    endMovement(chained_exit, target);
    chain_was_forward = false;
}

static void driveForward(const MotionRequest &request) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    bool continuing = last_was_chain; // the robot is still moving from a chained movement
    double target = request.target;
    double direction = getSign(target);
    // Measure the distance from where we start. Right after a chained PID_forward, measure from where
    // that one's target was instead: it handed over a bit early, and those inches must not get lost.
    double start_position = (continuing && chain_was_forward) ? chain_forward_end.load() : getPosition();
    // The heading we try to keep while driving. After a chained turn the robot is still turning,
    // so keep the heading that turn was going for instead of wherever it is right now.
    double start_heading = continuing ? chain_heading.load() : getInertial();

    // Chaining: aim a bit past the target, so the robot is still moving when it gets there
    bool chaining = request.exit_range > 0;
    double pid_target = chaining ? target + direction * request.exit_range : target;
    pid.reset(pid_target);
    telemetryStart("PID_forward", target, target, request.timeout_ms);

    bool chained_exit = false;
    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        double driven = getPosition() - start_position;
        double current_error = target - driven; // to the real target, not the chaining one
        double motor_rate = getMotorRate();
        motion_progress = fabs(driven);

        if (chaining) {
            // Close enough (or already past it): hand over to the next movement without stopping
            if (current_error * direction < request.exit_range) {
                chained_exit = true;
                break;
            }
        } else {
            bool inside = fabs(current_error) < request.error_tolerance && fabs(motor_rate) < request.speed_tolerance;
            if (settle.update(inside, time)) {
                break;
            }
        }

        double total_correction = cap(pid.compute(pid_target - driven, motor_rate), request.max_speed);
        if (!continuing) {
            total_correction = startRamp(total_correction, time); // no ramp if we are already moving
        }

        // Keep driving straight: if the robot turned clockwise, heading_correction is negative,
        // which slows the left side and speeds up the right side to turn back
        double heading_correction = (start_heading - getInertial()) * FORWARD_HEADING_KP;

        // (moveLimited keeps both sides under max_speed, heading correction included)
        moveLimited(total_correction + heading_correction, total_correction - heading_correction, request.max_speed);
        telemetryUpdate(time, current_error, motor_rate, total_correction, pid);
        vexDelay(delay);
    }
    endMovement(chained_exit, start_heading);
    chain_was_forward = chained_exit;
    chain_forward_end = start_position + target;
}

// Drive to a field point, using odometry. The robot keeps aiming at the point while it drives,
// so it gets there even if it gets bumped or one side is weaker.
static void driveToPoint(const MotionRequest &request) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    bool continuing = last_was_chain; // the robot is still moving from a chained movement
    double tx = request.target_x;
    double ty = request.target_y;
    double start_x = getX();
    double start_y = getY();
    double flip = request.backwards ? 180 : 0; // backwards: the back of the robot points at the target
    double heading = getInertial();
    double aim = heading + wrap180(headingTo(tx, ty) + flip - heading); // the heading to drive along
    double first_error = distanceAhead(tx, ty, heading);
    pid.reset(first_error);
    telemetryStart("PID_drive_to_point", hypot(tx - start_x, ty - start_y), first_error, request.timeout_ms);

    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            break; // took too long, give up so autonomous can continue
        }

        heading = getInertial();
        double ahead = distanceAhead(tx, ty, heading); // what is left to drive (negative = drove past it)
        double distance = hypot(tx - getX(), ty - getY());
        double motor_rate = getMotorRate();
        motion_progress = hypot(getX() - start_x, getY() - start_y);
        bool inside = fabs(ahead) < request.error_tolerance && fabs(motor_rate) < request.speed_tolerance;
        if (settle.update(inside, time)) {
            break;
        }

        // Keep aiming at the point, until we are close
        if (distance > POINT_AIM_DISTANCE) {
            aim = heading + wrap180(headingTo(tx, ty) + flip - heading);
        }
        double aim_error = aim - heading;

        double power = cap(pid.compute(ahead, motor_rate), request.max_speed);
        if (!continuing) {
            power = startRamp(power, time); // no ramp if we are already moving
        }
        // Not facing the point yet? Drive slower until the robot has turned towards it
        power *= fmax(cos(aim_error * M_PI / 180), 0);

        double heading_correction = aim_error * POINT_HEADING_KP;
        moveLimited(power + heading_correction, power - heading_correction, request.max_speed);
        telemetryUpdate(time, ahead, motor_rate, power, pid);
        vexDelay(delay);
    }
    endMovement(false, aim);
    chain_was_forward = false;
}

static void runMotion(const MotionRequest &request) {
    if (request.type == MOTION_FORWARD) {
        driveForward(request);
    } else if (request.type == MOTION_TO_POINT) {
        double start_time = Brain.timer(timeUnits::msec);
        double heading = getInertial();
        double flip = request.backwards ? 180 : 0;
        double turn_needed = wrap180(headingTo(request.target_x, request.target_y) + flip - heading);
        double distance = hypot(request.target_x - getX(), request.target_y - getY());
        if (fabs(turn_needed) > POINT_TURN_FIRST_ANGLE && distance > POINT_AIM_DISTANCE) {
            // Facing far away from the point: turn towards it first. The turn is chained (doesn't
            // stop at the end), the drive keeps correcting the aim anyway.
            MotionRequest turn = makeRequest(MOTION_TURN, heading + turn_needed, 0, 0, request.timeout_ms, request.max_speed);
            turn.exit_range = 5;
            turn.report_progress = false; // waitUntilTraveled counts inches driven, not degrees turned
            PIDController pid(TURN_KP, TURN_KI, TURN_KD, TURN_INTEGRAL_RANGE);
            turnToHeading(turn, pid, POINT_TURN, "PID_drive_to_point (turn)");
        }
        if (!cancel_requested) {
            MotionRequest drive = request;
            drive.timeout_ms = request.timeout_ms - (Brain.timer(timeUnits::msec) - start_time);
            driveToPoint(drive);
        }
    } else if (request.type == MOTION_TURN || (request.type == MOTION_ARC && fabs(request.radius) < 1)) {
        // (an arc with radius 0 is a turn in place)
        PIDController pid(TURN_KP, TURN_KI, TURN_KD, TURN_INTEGRAL_RANGE);
        turnToHeading(request, pid, POINT_TURN, "PID_turn");
    } else if (request.type == MOTION_ARC) {
        PIDController pid(ARC_KP, ARC_KI, ARC_KD, ARC_INTEGRAL_RANGE);
        turnToHeading(request, pid, ARC, "PID_arc");
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
        } else if (chain_end_time.load() >= 0 && Brain.timer(timeUnits::msec) - chain_end_time.load() > CHAIN_STOP_AFTER_MS) {
            // A chained movement ended but nothing followed it: don't leave the robot driving
            stopDriving();
            last_was_chain = false;
            chain_end_time = -1;
        }
        vexDelay(5);
    }
    return 0;
}

static MotionRequest makeRequest(motionType type, double target, double error_tolerance, double speed_tolerance,
                                 double timeout_ms, double max_speed) {
    MotionRequest request;
    request.type = type;
    request.target = target;
    request.error_tolerance = error_tolerance;
    request.speed_tolerance = speed_tolerance;
    request.timeout_ms = timeout_ms;
    request.max_speed = max_speed;
    request.radius = 0;
    request.exit_range = 0;
    request.target_x = 0;
    request.target_y = 0;
    request.backwards = false;
    request.report_progress = true;
    return request;
}

// Same, for a chained movement: it finishes exit_range before the target, without stopping
static MotionRequest makeChainRequest(motionType type, double target, double exit_range,
                                      double timeout_ms, double max_speed) {
    MotionRequest request = makeRequest(type, target, 0, 0, timeout_ms, max_speed);
    request.exit_range = fmax(exit_range, 0.01); // must be above 0, or it would not count as chained
    return request;
}

// Hand a movement to the background task. Waits for the previous movement to finish first.
static void startMotion(const MotionRequest &request) {
    // Claim the drivetrain: wait until no movement is running, then mark it as running in one
    // step (compare_exchange), so two tasks can never start a movement at the same moment
    bool expected = false;
    while (!motion_running.compare_exchange_weak(expected, true)) {
        expected = false;
        vexDelay(5);
    }
    if (motion_task == nullptr) {
        // Made with "new" so the task object is never destroyed and the task keeps running
        motion_task = new task(motionLoop);
    }
    next_motion = request;
    cancel_requested = false;
    motion_progress = 0;
    motion_requested = true; // the background task sees next_motion once it sees this
}

// The heading to measure relative and shortest-way turns from. Right after a chained movement
// the robot is still turning, so use the heading that movement was going for.
static double currentHeadingForTurns() {
    return last_was_chain ? chain_heading.load() : getInertial();
}

static double relativeTarget(double degrees) {
    return currentHeadingForTurns() + degrees;
}

static double shortestTarget(double heading) {
    double current = currentHeadingForTurns();
    return current + wrap180(heading - current); // always the short way around
}

static motionType swingType(driveSide moving_side) {
    return (moving_side == LEFT_SIDE) ? MOTION_SWING_LEFT : MOTION_SWING_RIGHT;
}

static MotionRequest makeArcRequest(double target, double radius, double error_tolerance, double speed_tolerance,
                                    double timeout_ms, double max_speed) {
    MotionRequest request = makeRequest(MOTION_ARC, target, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    request.radius = radius;
    return request;
}

// ---------- Start a movement and return right away ----------

void PID_forward_async(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    startMotion(makeRequest(MOTION_FORWARD, target, error_tolerance, speed_tolerance, timeout_ms, max_speed));
}

void PID_turn_async(double target, double error_tolerance, double speed_tolerance, double timeout_ms, double max_speed) {
    startMotion(makeRequest(MOTION_TURN, target, error_tolerance, speed_tolerance, timeout_ms, max_speed));
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
    startMotion(makeRequest(swingType(moving_side), target, error_tolerance, speed_tolerance, timeout_ms, max_speed));
}

void PID_arc_async(double target, double radius, double error_tolerance, double speed_tolerance,
                   double timeout_ms, double max_speed) {
    startMotion(makeArcRequest(target, radius, error_tolerance, speed_tolerance, timeout_ms, max_speed));
}

void PID_turn_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                             double timeout_ms, double max_speed, bool backwards) {
    startOdometry();
    waitUntilDone(); // aim from where the robot is after the previous movement ends
    double flip = backwards ? 180 : 0;
    PID_turn_async(shortestTarget(headingTo(x, y) + flip), error_tolerance, speed_tolerance, timeout_ms, max_speed);
}

void PID_drive_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                              double timeout_ms, double max_speed, bool backwards) {
    startOdometry();
    MotionRequest request = makeRequest(MOTION_TO_POINT, 0, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    request.target_x = x;
    request.target_y = y;
    request.backwards = backwards;
    startMotion(request);
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

void PID_arc(double target, double radius, double error_tolerance, double speed_tolerance,
             double timeout_ms, double max_speed) {
    PID_arc_async(target, radius, error_tolerance, speed_tolerance, timeout_ms, max_speed);
    waitUntilDone();
}

void PID_turn_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                       double timeout_ms, double max_speed, bool backwards) {
    PID_turn_to_point_async(x, y, error_tolerance, speed_tolerance, timeout_ms, max_speed, backwards);
    waitUntilDone();
}

void PID_drive_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                        double timeout_ms, double max_speed, bool backwards) {
    PID_drive_to_point_async(x, y, error_tolerance, speed_tolerance, timeout_ms, max_speed, backwards);
    waitUntilDone();
}

// ---------- Chained: finish early without stopping, so the next movement continues smoothly ----------

void PID_forward_chain(double target, double exit_range, double timeout_ms, double max_speed) {
    startMotion(makeChainRequest(MOTION_FORWARD, target, exit_range, timeout_ms, max_speed));
    waitUntilDone();
}

void PID_turn_chain(double target, double exit_range, double timeout_ms, double max_speed) {
    startMotion(makeChainRequest(MOTION_TURN, target, exit_range, timeout_ms, max_speed));
    waitUntilDone();
}

void PID_swing_chain(double target, driveSide moving_side, double exit_range, double timeout_ms, double max_speed) {
    startMotion(makeChainRequest(swingType(moving_side), target, exit_range, timeout_ms, max_speed));
    waitUntilDone();
}

void PID_arc_chain(double target, double radius, double exit_range, double timeout_ms, double max_speed) {
    MotionRequest request = makeChainRequest(MOTION_ARC, target, exit_range, timeout_ms, max_speed);
    request.radius = radius;
    startMotion(request);
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
    if (motion_running) {
        cancel_requested = true;
        waitUntilDone();
        cancel_requested = false;
    }
    if (last_was_chain) {
        // A chained movement ended and the motors are still running
        stopDriving();
        last_was_chain = false;
        chain_end_time = -1;
    }
}
