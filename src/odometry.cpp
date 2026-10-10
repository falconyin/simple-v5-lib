#include "simpleV5lib.h"
#include <atomic>

// Odometry: a background task that keeps track of where the robot is on the field.
//
// Every 10 ms it reads how far the robot drove forward (and sideways, with a sideways
// tracking wheel) and how much it turned since the last time, and adds that piece of
// movement to the position. Field coordinates, in inches:
//   x: to the right       y: forward (the direction heading 0 points)
// Heading comes straight from the inertial sensor (getInertial: clockwise is positive).

static std::atomic<double> pose_x(0);
static std::atomic<double> pose_y(0);
// Goes up by one every time the heading or position is changed by hand (setHeading, setPose).
// When the odometry loop sees it change, its next reading is a new starting point instead of a
// movement, otherwise a heading jump would look like the robot turning. A counter instead of a
// yes/no flag, so a change can never get lost, however the two tasks happen to take turns.
static std::atomic<unsigned> manual_changes(0);

static std::atomic<bool> odometry_started(false);
static rotation* forward_wheel = nullptr;  // tracking wheels, nullptr = not used
static rotation* sideways_wheel = nullptr;

// How far a tracking wheel has rolled, in inches
static double wheelInches(rotation* wheel) {
    return wheel->position(rotationUnits::rev) * TRACKING_WHEEL_DIAMETER_INCH * M_PI;
}

static int odometryLoop() {
    double last_forward = 0;
    double last_sideways = 0;
    double last_heading = 0;
    bool have_start = false;
    unsigned changes_seen = 0;

    while (true) {
        unsigned changes_before = manual_changes;
        double forward = (forward_wheel != nullptr) ? wheelInches(forward_wheel) : getPosition();
        double sideways = (sideways_wheel != nullptr) ? wheelInches(sideways_wheel) : 0;
        double heading = getInertial();
        unsigned changes_after = manual_changes;

        if (!have_start || changes_before != changes_seen || changes_after != changes_before) {
            // First reading, or the pose was changed by hand: start counting from here.
            // (If it changed while we were reading, changes_seen stays behind and the next
            // reading starts over again, with values that are surely after the change.)
            have_start = true;
            changes_seen = changes_before;
        } else {
            double turned = (heading - last_heading) * M_PI / 180; // radians, clockwise is positive
            double moved_forward = forward - last_forward;
            double moved_sideways = sideways - last_sideways;

            // A tracking wheel that is not in the middle of the robot also rolls when the robot
            // turns in place. Take that part out, so we get how far the middle of the robot moved.
            // (The drive motors are averaged over the left and right side, which already is the middle.)
            if (forward_wheel != nullptr) {
                moved_forward += TRACKING_FORWARD_OFFSET * turned;
            }
            if (sideways_wheel != nullptr) {
                moved_sideways -= TRACKING_SIDEWAYS_OFFSET * turned;
            }

            // While turning, the robot drove along a curve. The straight line from the old to the
            // new position (the "chord") is a little shorter than the curve.
            if (fabs(turned) > 1e-9) {
                double chord = 2 * sin(turned / 2) / turned;
                moved_forward *= chord;
                moved_sideways *= chord;
            }

            // The robot faced in between the old and new heading on the way: turn the movement
            // from "forward / sideways of the robot" into "x / y on the field"
            double facing = last_heading * M_PI / 180 + turned / 2;
            pose_x = pose_x + moved_forward * sin(facing) + moved_sideways * cos(facing);
            pose_y = pose_y + moved_forward * cos(facing) - moved_sideways * sin(facing);
        }

        last_forward = forward;
        last_sideways = sideways;
        last_heading = heading;
        vexDelay(10);
    }
    return 0;
}

void startOdometry() {
    // exchange sets it to true and tells us what it was: only the very first call gets "false",
    // so two tasks calling this at the same time can never start two odometry loops
    if (odometry_started.exchange(true)) {
        return; // already running
    }
    if (TRACKING_FORWARD_PORT >= 0) {
        forward_wheel = new rotation(TRACKING_FORWARD_PORT, TRACKING_FORWARD_REVERSED);
    }
    if (TRACKING_SIDEWAYS_PORT >= 0) {
        sideways_wheel = new rotation(TRACKING_SIDEWAYS_PORT, TRACKING_SIDEWAYS_REVERSED);
    }
    // Made with "new" so the task object is never destroyed and the task keeps running
    new task(odometryLoop);
}

// In simpleV5lib.cpp: keeps a chained movement's heading in step when the heading jumps
void shiftChainHeading(double degrees);

void setHeading(double degrees) {
    manual_changes++; // the heading jumps now: don't count that as the robot turning
    shiftChainHeading(degrees - getInertial());
    Inertial.setRotation(degrees, rotationUnits::deg);
    // heading() only goes from 0 to 360, so wrap the value into that range
    double heading = fmod(degrees, 360);
    if (heading < 0) {
        heading += 360;
    }
    Inertial.setHeading(heading, rotationUnits::deg);
}

void setPose(double x, double y, double heading) {
    startOdometry();
    setHeading(heading);
    pose_x = x;
    pose_y = y;
    manual_changes++;
}

void setX(double x) {
    startOdometry();
    // Only x changes: the wheel and heading readings go on as before, so no fresh start is needed
    pose_x = x;
}

void setY(double y) {
    startOdometry();
    pose_y = y;
}

// The V5 distance sensor can't measure closer than 20 mm
const double DISTANCE_SENSOR_MIN_INCH = 0.8;

// How fast one side of the drivetrain is moving, in inches/s
static double wheelSpeed(motor &m) {
    return m.velocity(velocityUnits::dps) / 360 * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO;
}

// Shared by resetXFromWall and resetYFromWall. x_wall is true for a wall at x = wall,
// false for a wall at y = wall.
static bool resetFromWall(distanceSensor which, bool x_wall, double wall, double max_change) {
    const char* axis = x_wall ? "x" : "y";
    if (which < FRONT_SENSOR || which > RIGHT_SENSOR) {
        printf("reset %s from wall: unknown sensor %d\n", axis, (int)which);
        return false;
    }
    // Where each sensor sits on the robot (from simpleV5LibConfig.h) and which way it looks,
    // in degrees clockwise from the robot's front
    struct Mount { const char* name; int port; double ahead; double right; double looks; };
    const Mount mounts[] = {
        {"front", DISTANCE_FRONT_PORT, DISTANCE_FRONT_AHEAD, DISTANCE_FRONT_RIGHT, 0},
        {"back", DISTANCE_BACK_PORT, DISTANCE_BACK_AHEAD, DISTANCE_BACK_RIGHT, 180},
        {"left", DISTANCE_LEFT_PORT, DISTANCE_LEFT_AHEAD, DISTANCE_LEFT_RIGHT, -90},
        {"right", DISTANCE_RIGHT_PORT, DISTANCE_RIGHT_AHEAD, DISTANCE_RIGHT_RIGHT, 90},
    };
    const Mount &mount = mounts[which];

    if (mount.port < 0) {
        printf("reset %s from wall: no %s distance sensor in simpleV5LibConfig.h\n", axis, mount.name);
        return false;
    }
    // The sensor's reading is a little behind: while the robot moves, it belongs to where the
    // robot was a moment ago, not to where odometry says it is now
    if (fmax(fabs(wheelSpeed(leftFront)), fabs(wheelSpeed(rightFront))) > DISTANCE_RESET_MAX_SPEED) {
        printf("reset %s from wall: the robot is still moving\n", axis);
        return false;
    }
    distance sensor(mount.port);
    if (!sensor.installed()) {
        printf("reset %s from wall: no distance sensor on port %d\n", axis, mount.port + 1);
        return false;
    }
    if (!sensor.isObjectDetected()) {
        printf("reset %s from wall: the %s sensor doesn't see anything\n", axis, mount.name);
        return false;
    }
    double reading = sensor.objectDistance(distanceUnits::in);
    if (reading < DISTANCE_SENSOR_MIN_INCH) {
        printf("reset %s from wall: the %s sensor is too close to the wall to measure\n", axis, mount.name);
        return false;
    }

    // Which way the sensor looks on the field. beam_along: how much of each inch along the beam
    // goes in the x (or y) direction (heading 0 looks along +y, clockwise is positive)
    double heading = getInertial() * M_PI / 180;
    double beam = heading + mount.looks * M_PI / 180;
    double beam_along = x_wall ? sin(beam) : cos(beam);

    // A wall at x = ... is straight on when the beam goes only along x. At an angle, part of
    // every inch goes sideways, and fabs(beam_along) is the cos of the angle from straight on.
    if (fabs(beam_along) < cos(DISTANCE_RESET_MAX_ANGLE * M_PI / 180)) {
        printf("reset %s from wall: the %s sensor doesn't look at the wall straight enough\n", axis, mount.name);
        return false;
    }

    // The wall must be on the side the sensor looks at (bigger x when the beam goes towards
    // bigger x). If not, it's the wrong sensor or the wrong wall: the sensor sees a different one.
    double old_value = x_wall ? getX() : getY();
    if ((wall - old_value) * beam_along <= 0) {
        printf("reset %s from wall: the wall at %s = %.1f is behind the %s sensor\n", axis, axis, wall, mount.name);
        return false;
    }

    // Where the sensor is, measured from the robot's center, on the field
    double sensor_x = mount.ahead * sin(heading) + mount.right * cos(heading);
    double sensor_y = mount.ahead * cos(heading) - mount.right * sin(heading);

    // Walk back from the wall: along the beam to the sensor, then from the sensor to the center.
    // (beam_along is negative when the beam looks towards smaller x or y, so this works for
    // the walls on both sides.)
    double new_value = wall - reading * beam_along - (x_wall ? sensor_x : sensor_y);
    if (fabs(new_value - old_value) > max_change) {
        printf("reset %s from wall: the %s sensor says %s = %.1f, but odometry says %.1f. Not the wall?\n",
               axis, mount.name, axis, new_value, old_value);
        return false;
    }
    if (x_wall) {
        setX(new_value);
    } else {
        setY(new_value);
    }
    return true;
}

bool resetXFromWall(distanceSensor sensor, double wall_x, double max_change) {
    return resetFromWall(sensor, true, wall_x, max_change);
}

bool resetYFromWall(distanceSensor sensor, double wall_y, double max_change) {
    return resetFromWall(sensor, false, wall_y, max_change);
}

double getForwardTrackingWheel() {
    return (forward_wheel != nullptr) ? wheelInches(forward_wheel) : 0;
}

double getSidewaysTrackingWheel() {
    return (sideways_wheel != nullptr) ? wheelInches(sideways_wheel) : 0;
}

double getX() {
    return pose_x;
}

double getY() {
    return pose_y;
}
