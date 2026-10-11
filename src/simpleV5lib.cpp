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

// The gains the movements use. They start with the values from simpleV5LibConfig.h.
PIDGains turnGains = {TURN_KP, TURN_KI, TURN_KD};
PIDGains swingGains = {SWING_KP, SWING_KI, SWING_KD};
PIDGains forwardGains = {FORWARD_KP, FORWARD_KI, FORWARD_KD};
PIDGains arcGains = {ARC_KP, ARC_KI, ARC_KD};

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
static double last_drive_power = 0; // the average of the two sides, the last time move() was called
void move(double left_speed, double right_speed) {
    double biggest = fmax(fabs(left_speed), fabs(right_speed));
    if (biggest > 100) {
        left_speed = left_speed / biggest * 100;
        right_speed = right_speed / biggest * 100;
    }
    spinSide(leftDrive, left_speed);
    spinSide(rightDrive, right_speed);
    last_drive_power = (left_speed + right_speed) / 2;
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
    // A copy of the gains, taken in your task when the movement starts. The background task only
    // uses this copy, so changing turnGains & co. can never mix up a movement that is running.
    PIDGains turn_gains;
    PIDGains swing_gains;
    PIDGains forward_gains;
    PIDGains arc_gains;
};

static PIDController makePID(const PIDGains &gains, double integral_range) {
    return PIDController(gains.kp, gains.ki, gains.kd, integral_range);
}

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
// How the last movement went. Written by the background task before it sets motion_running to
// false, read by lastMovementResult() after waiting for that, so the two never overlap.
static MovementResult last_result = {0, 0, 0, false};

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

// Keeps track of how far a movement went past its target. Call update() once per loop with the
// error and the direction the movement goes (+1 or -1): a negative error * direction means past it.
struct OvershootCheck {
    double most = 0;

    void update(double error, double direction) {
        most = fmax(most, -error * direction);
    }
};

static void saveResult(double start_time, double error, double overshoot, bool timed_out) {
    last_result.time_ms = Brain.timer(timeUnits::msec) - start_time;
    last_result.error = error;
    last_result.overshoot = overshoot;
    last_result.timed_out = timed_out;
}

// Speed up gently so the wheels don't slip: the power can't go above a limit that starts at 30%,
// or at the power the drivetrain already had the same way (from_power: after a chained movement),
// and rises by RAMP_PERCENT_PER_S. Only limits how hard it pushes, the direction comes from the PID.
// After a chained turn in place the robot is turning, but not driving yet (its average power is
// about 0), and when the new movement goes the other way it must reverse: both start from 30%.
const double RAMP_PERCENT_PER_S = 233;
static double startRamp(double power, double time_ms, double from_power) {
    double from = 30;
    if (getSign(from_power) == getSign(power) && fabs(from_power) > from) {
        from = fabs(from_power);
    }
    double ramp_limit = from + (time_ms / 1000 * RAMP_PERCENT_PER_S);
    if (fabs(power) > ramp_limit) {
        return getSign(power) * ramp_limit;
    }
    return power;
}

// Slower than this (inches/s), the robot counts as not driving: a turn in place can begin
const double WALKING_SPEED = 5;
// Before a chained drive to a point curves into a sharp corner, it slows down to this (inches/s)
const double CORNER_SPEED = 20;
// Faster than this (inches/s), the robot counts as already driving: no gentle start needed
const double ALREADY_DRIVING_SPEED = 25;

// Where startRamp starts for a movement that drives in the direction travel (+1 forward, -1 backwards).
// After a chained movement the robot is still moving. Still driving fast that way: full power is fine,
// the wheels already turn nearly that fast. Slower (near the end of a chained movement the PID has
// eased off, its D term brakes, and slowDown() brings the power down to a trickle): from whatever the
// drivetrain was pushing with. After a normal movement the robot stands still: from the start.
static double rampFrom(double travel) {
    if (!last_was_chain) {
        return 0;
    }
    if (getMotorRate() * travel > ALREADY_DRIVING_SPEED) {
        return travel * 100;
    }
    return last_drive_power;
}

// Slows the robot down to to_speed before a turn in place or a sharp corner while it still rolls from
// a chained movement. Wheels that reverse at speed skid, and the drive motors count the skidding as
// driving, which throws getPosition() and odometry off. Braking hard would skid too, so this is
// startRamp the other way round: the power comes down at the same gentle rate, from whatever the
// last movement was pushing with. Gives up after SLOW_DOWN_TIMEOUT_MS, so a wheel that keeps
// reporting speed (lifted off the ground?) can't hold the next movement forever.
const double SLOW_DOWN_TIMEOUT_MS = 800;
static void slowDown(double to_speed) {
    double start_time = Brain.timer(timeUnits::msec);
    double from = last_drive_power;
    while (!cancel_requested && fabs(getMotorRate()) > to_speed) {
        double time_ms = Brain.timer(timeUnits::msec) - start_time;
        if (time_ms > SLOW_DOWN_TIMEOUT_MS) {
            break;
        }
        double power = fmax(0, fabs(from) - time_ms / 1000 * RAMP_PERCENT_PER_S);
        move(getSign(from) * power, getSign(from) * power);
        vexDelay(10);
    }
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
    if (style != ARC) {
        // Turning in place or swinging while still rolling (after a chained movement) would skid.
        // Before the clock starts: the turn's time and log count from when the turning begins.
        slowDown(WALKING_SPEED);
    }
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    SettleCheck settle(TURN_SETTLE_MS);
    double ramp_from = rampFrom(getSign(request.radius)); // (arcs only) still driving from a chained movement
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
    bool timed_out = false;
    OvershootCheck overshoot;
    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            timed_out = true;
            break; // took too long, give up so autonomous can continue
        }

        double current_heading = getInertial();
        double current_error = target - current_heading; // to the real target, not the chaining one
        double gyro_rate = getGyroRate();
        overshoot.update(current_error, direction);
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
        if (style == ARC) {
            total_correction = startRamp(total_correction, time, ramp_from);
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
            last_drive_power = total_correction / 2; // one side pushes, the other stands: half the power on average
        } else if (style == RIGHT_SWING) {
            spinSide(rightDrive, total_correction * -1); // right side backward = clockwise
            leftDrive.stop(brakeType::hold);
            last_drive_power = total_correction * -1 / 2;
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
    saveResult(start_time, target - getInertial(), overshoot.most, timed_out);
    endMovement(chained_exit, target);
    chain_was_forward = false;
}

static void driveForward(const MotionRequest &request) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid = makePID(request.forward_gains, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    bool continuing = last_was_chain; // the robot is still moving from a chained movement
    double target = request.target;
    double direction = getSign(target);
    double ramp_from = rampFrom(direction); // ...and if it drives this way already, no fresh start
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
    bool timed_out = false;
    OvershootCheck overshoot;
    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            timed_out = true;
            break; // took too long, give up so autonomous can continue
        }

        double driven = getPosition() - start_position;
        double current_error = target - driven; // to the real target, not the chaining one
        double motor_rate = getMotorRate();
        motion_progress = fabs(driven);
        overshoot.update(current_error, direction);

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
        total_correction = startRamp(total_correction, time, ramp_from);

        // Keep driving straight: if the robot turned clockwise, heading_correction is negative,
        // which slows the left side and speeds up the right side to turn back
        double heading_correction = (start_heading - getInertial()) * FORWARD_HEADING_KP;

        // (moveLimited keeps both sides under max_speed, heading correction included)
        moveLimited(total_correction + heading_correction, total_correction - heading_correction, request.max_speed);
        telemetryUpdate(time, current_error, motor_rate, total_correction, pid);
        vexDelay(delay);
    }
    saveResult(start_time, target - (getPosition() - start_position), overshoot.most, timed_out);
    endMovement(chained_exit, start_heading);
    chain_was_forward = chained_exit;
    chain_forward_end = start_position + target;
}

// While driving to a point, the robot only counts as finished when it faces the way it is aiming
// within this many degrees. Otherwise a point right beside the robot is "0 inches ahead" before
// the robot has even turned towards it.
const double POINT_FACING_TOLERANCE = 5;

// A chained drive to a point, started while the robot is still moving from a chained movement,
// curves towards its point instead of turning first, unless the point is more than this many
// degrees away from where the robot faces
const double POINT_CURVE_ANGLE = 90;

// Drive to a field point, using odometry. The robot keeps aiming at the point while it drives,
// so it gets there even if it gets bumped or one side is weaker.
static void driveToPoint(const MotionRequest &request) {
    double start_time = Brain.timer(timeUnits::msec);
    long delay = 10;
    PIDController pid = makePID(request.forward_gains, FORWARD_INTEGRAL_RANGE);
    SettleCheck settle(FORWARD_SETTLE_MS);
    double tx = request.target_x;
    double ty = request.target_y;
    double start_x = getX();
    double start_y = getY();
    double flip = request.backwards ? 180 : 0; // backwards: the back of the robot points at the target
    double heading = getInertial();
    double aim = heading + wrap180(headingTo(tx, ty) + flip - heading); // the heading to drive along
    double first_error = distanceAhead(tx, ty, heading);
    double travel = request.backwards ? -1 : 1; // the way the robot drives: +1 forward, -1 backwards
    double ramp_from = rampFrom(travel); // still driving this way from a chained movement: no fresh start

    // Chaining: aim a bit past the point, so the robot is still moving when it gets there
    bool chaining = request.exit_range > 0;
    double pid_offset = chaining ? travel * request.exit_range : 0;
    pid.reset(first_error + pid_offset);
    telemetryStart("PID_drive_to_point", hypot(tx - start_x, ty - start_y), first_error, request.timeout_ms);

    bool chained_exit = false;
    bool timed_out = false;
    OvershootCheck overshoot;
    while (!cancel_requested) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > request.timeout_ms) {
            timed_out = true;
            break; // took too long, give up so autonomous can continue
        }

        heading = getInertial();
        double ahead = distanceAhead(tx, ty, heading); // what is left to drive (negative = drove past it)
        double distance = hypot(tx - getX(), ty - getY());
        double motor_rate = getMotorRate();
        motion_progress = hypot(getX() - start_x, getY() - start_y);
        // Driving forward, a negative "ahead" means the robot drove past the point; backwards, a positive one
        overshoot.update(ahead, travel);

        // Keep aiming at the point, until we are close
        if (distance > POINT_AIM_DISTANCE) {
            aim = heading + wrap180(headingTo(tx, ty) + flip - heading);
        }
        double aim_error = aim - heading;

        if (chaining) {
            // Close enough (or already past it), and roughly facing the way we aim: hand over to the
            // next movement without stopping. (Without the facing check, a point right beside the
            // robot would be "0 inches ahead" and the movement would end before it even started.)
            if (ahead * travel < request.exit_range && fabs(aim_error) < POINT_TURN_FIRST_ANGLE) {
                chained_exit = true;
                break;
            }
        } else {
            // Finished: nothing left to drive, stopped, and facing the way we aim. (The robot can't drive
            // sideways, so a little sideways miss is fine; the straight-line distance could never settle.)
            bool inside = fabs(ahead) < request.error_tolerance && fabs(motor_rate) < request.speed_tolerance
                          && fabs(aim_error) < POINT_FACING_TOLERANCE;
            if (settle.update(inside, time)) {
                break;
            }
        }

        double power = cap(pid.compute(ahead + pid_offset, motor_rate), request.max_speed);
        power = startRamp(power, time, ramp_from);
        // Not facing the point yet? Drive slower until the robot has turned towards it
        power *= fmax(cos(aim_error * M_PI / 180), 0);

        double heading_correction = aim_error * POINT_HEADING_KP;
        moveLimited(power + heading_correction, power - heading_correction, request.max_speed);
        telemetryUpdate(time, ahead, motor_rate, power, pid);
        vexDelay(delay);
    }
    saveResult(start_time, distanceAhead(tx, ty, getInertial()), overshoot.most, timed_out);
    endMovement(chained_exit, aim);
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
        // In the middle of a chain the robot is still driving: curving into the new direction is much
        // faster than turning on the spot (driveToPoint steers and slows down as needed). The last,
        // normal movement of a chain does turn first: it drives straight in, so it stops exactly.
        bool curve = last_was_chain && request.exit_range > 0 && fabs(turn_needed) <= POINT_CURVE_ANGLE;
        if (curve && fabs(turn_needed) > POINT_TURN_FIRST_ANGLE) {
            // A sharp corner at full speed would skid the inside wheels: slow down a bit first
            slowDown(CORNER_SPEED);
        }
        if (fabs(turn_needed) > POINT_TURN_FIRST_ANGLE && distance > POINT_AIM_DISTANCE && !curve) {
            // Facing far away from the point: turn towards it first. The turn is chained (doesn't
            // stop at the end), the drive keeps correcting the aim anyway.
            MotionRequest turn = makeRequest(MOTION_TURN, heading + turn_needed, 0, 0, request.timeout_ms, request.max_speed);
            turn.exit_range = 5;
            turn.report_progress = false; // waitUntilTraveled counts inches driven, not degrees turned
            turnToHeading(turn, makePID(request.turn_gains, TURN_INTEGRAL_RANGE), POINT_TURN, "PID_drive_to_point (turn)");
        }
        if (!cancel_requested) {
            MotionRequest drive = request;
            drive.timeout_ms = request.timeout_ms - (Brain.timer(timeUnits::msec) - start_time);
            driveToPoint(drive);
        } else {
            // Cancelled before driving: the result so far is the turn's (in degrees), or, without a
            // turn, still the previous movement's. Report it like a drive to a point instead:
            // inches still left to drive, it never went past the point, and it didn't time out.
            last_result.error = distanceAhead(request.target_x, request.target_y, getInertial());
            last_result.overshoot = 0;
            last_result.timed_out = false;
        }
        last_result.time_ms = Brain.timer(timeUnits::msec) - start_time; // the turn counts too
    } else if (request.type == MOTION_TURN || (request.type == MOTION_ARC && fabs(request.radius) < 1)) {
        // (an arc with radius 0 is a turn in place)
        turnToHeading(request, makePID(request.turn_gains, TURN_INTEGRAL_RANGE), POINT_TURN, "PID_turn");
    } else if (request.type == MOTION_ARC) {
        turnToHeading(request, makePID(request.arc_gains, ARC_INTEGRAL_RANGE), ARC, "PID_arc");
    } else {
        turnStyle style = (request.type == MOTION_SWING_LEFT) ? LEFT_SWING : RIGHT_SWING;
        turnToHeading(request, makePID(request.swing_gains, SWING_INTEGRAL_RANGE), style, "PID_swing");
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
        } else if (chain_end_time.load() < 0) {
            // Nothing is moving: a good moment for the slow SD card (logToSDCard)
            telemetryWriteSDCard();
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
    next_motion.turn_gains = turnGains;
    next_motion.swing_gains = swingGains;
    next_motion.forward_gains = forwardGains;
    next_motion.arc_gains = arcGains;
    cancel_requested = false;
    motion_progress = 0;
    motion_requested = true; // the background task sees next_motion once it sees this
}

// The heading to measure relative and shortest-way turns from. Right after a chained movement
// the robot is still turning, so use the heading that movement was going for.
static double currentHeadingForTurns() {
    return last_was_chain ? chain_heading.load() : getInertial();
}

// Called by setHeading: the heading numbers just jumped by this many degrees. Move the heading a
// chained movement was going for by the same amount, so the next movement still aims at the same
// real direction (otherwise it would turn the robot back to the old number).
void shiftChainHeading(double degrees) {
    chain_heading = chain_heading + degrees;
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

void PID_drive_to_point_chain(double x, double y, double exit_range, double timeout_ms, double max_speed, bool backwards) {
    startOdometry();
    MotionRequest request = makeChainRequest(MOTION_TO_POINT, 0, exit_range, timeout_ms, max_speed);
    request.target_x = x;
    request.target_y = y;
    request.backwards = backwards;
    startMotion(request);
    waitUntilDone();
}

void PID_turn_to_point_chain(double x, double y, double exit_range, double timeout_ms, double max_speed, bool backwards) {
    startOdometry();
    waitUntilDone(); // aim from where the robot is after the previous movement ends
    double flip = backwards ? 180 : 0;
    PID_turn_chain(shortestTarget(headingTo(x, y) + flip), exit_range, timeout_ms, max_speed);
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

MovementResult lastMovementResult() {
    waitUntilDone(); // the background task writes the result right before the movement counts as done
    return last_result;
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
