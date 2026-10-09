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
// Set when the heading or position is changed by hand: the next reading is a new starting
// point instead of a movement (otherwise a heading jump would look like the robot turning)
static std::atomic<bool> start_over(true);

static task* odometry_task = nullptr;
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

    while (true) {
        double forward = (forward_wheel != nullptr) ? wheelInches(forward_wheel) : getPosition();
        double sideways = (sideways_wheel != nullptr) ? wheelInches(sideways_wheel) : 0;
        double heading = getInertial();

        if (start_over) {
            start_over = false;
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
    if (odometry_task != nullptr) {
        return; // already running
    }
    if (TRACKING_FORWARD_PORT >= 0) {
        forward_wheel = new rotation(TRACKING_FORWARD_PORT, TRACKING_FORWARD_REVERSED);
    }
    if (TRACKING_SIDEWAYS_PORT >= 0) {
        sideways_wheel = new rotation(TRACKING_SIDEWAYS_PORT, TRACKING_SIDEWAYS_REVERSED);
    }
    start_over = true;
    // Made with "new" so the task object is never destroyed and the task keeps running
    odometry_task = new task(odometryLoop);
}

void setHeading(double degrees) {
    start_over = true; // the heading jumps now: don't count that as the robot turning
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
    start_over = true;
}

double getX() {
    return pose_x;
}

double getY() {
    return pose_y;
}
