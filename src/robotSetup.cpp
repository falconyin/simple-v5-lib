#include "simpleV5lib.h"
#include <cstdarg>
#include <cstdio>

// Robot setup checks and measurements (checkDevices, testDrivetrain, measureTrackWidth,
// measureWheelSize). What they find is shown on the Brain screen and printed to the terminal
// (connect with a USB cable to see it), and a short message goes to the controller.

// ============================================================================
// Showing results
// ============================================================================

static int screen_row = 1;

static void startReport(const char* title) {
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(color::white); // the graph (graphOnScreen) may have left another color
    screen_row = 1;
    printf("\n== %s ==\n", title);
    Brain.Screen.setCursor(screen_row, 1);
    Brain.Screen.print("%s", title);
    screen_row += 2; // an empty line under the title
}

// One line on the Brain screen and in the terminal, used like printf.
// The Brain screen has room for about 12 lines of 48 letters.
static void report(const char* format, ...) {
    char line[100];
    va_list values;
    va_start(values, format);
    vsnprintf(line, sizeof(line), format, values);
    va_end(values);
    printf("%s\n", line);
    fflush(stdout);
    Brain.Screen.setCursor(screen_row, 1);
    Brain.Screen.print("%s", line);
    screen_row++;
}

// A short message on the controller screen (2 lines of up to 19 letters)
static void controllerMessage(const char* line1, const char* line2) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("%s", line1);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("%s", line2);
}

// Wait until A is pressed (or the Brain screen is tapped), and let go again
static void waitForPress() {
    while (!Controller.ButtonA.pressing() && !Brain.Screen.pressing()) {
        vexDelay(10);
    }
    while (Controller.ButtonA.pressing() || Brain.Screen.pressing()) {
        vexDelay(10);
    }
}

// ============================================================================
// The devices from simpleV5LibConfig.h
// ============================================================================

struct DriveMotor {
    const char* name;
    motor* device;
    int port;
    bool left;           // on the left side of the drivetrain
    const char* setting; // the direction setting in simpleV5LibConfig.h
};

static DriveMotor drive_motors[] = {
    {"left front",   &leftFront,   PORT_LEFTFRONT,   true,  "LF_DIRECTION"},
    {"left middle",  &leftMiddle,  PORT_LEFTMIDDLE,  true,  "LM_DIRECTION"},
    {"left back",    &leftBack,    PORT_LEFTBACK,    true,  "LB_DIRECTION"},
    {"right front",  &rightFront,  PORT_RIGHTFRONT,  false, "RF_DIRECTION"},
    {"right middle", &rightMiddle, PORT_RIGHTMIDDLE, false, "RM_DIRECTION"},
    {"right back",   &rightBack,   PORT_RIGHTBACK,   false, "RB_DIRECTION"},
};
const int DRIVE_MOTOR_COUNT = 6;

// How far one drive motor has turned its wheel, in inches
static double motorInches(motor &m) {
    return m.position(rotationUnits::rev) * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO;
}

// Real wheel size / the size in simpleV5LibConfig.h, from the last measureWheelSize() in this
// program run (1 = not measured yet). measureTrackWidth uses them: with wheels that are really a
// bit bigger, every inch the library counts is really a bit more, and so is the track width.
static double drive_wheel_scale = 1;
static double tracking_wheel_scale = 1;

// ============================================================================
// checkDevices
// ============================================================================

// V5 motors start to lose power when they get hotter than this (degrees Celsius)
const double MOTOR_HOT_CELSIUS = 55;

bool checkDevices() {
    startReport("Device check");
    int problems = 0;

    // Two devices set to the same port in simpleV5LibConfig.h
    struct Device { const char* name; int port; };
    Device devices[] = {
        {"left front motor", PORT_LEFTFRONT},     {"left middle motor", PORT_LEFTMIDDLE},
        {"left back motor", PORT_LEFTBACK},       {"right front motor", PORT_RIGHTFRONT},
        {"right middle motor", PORT_RIGHTMIDDLE}, {"right back motor", PORT_RIGHTBACK},
        {"inertial sensor", PORT_INERTIAL},
        {"forward tracking wheel", TRACKING_FORWARD_PORT}, {"sideways tracking wheel", TRACKING_SIDEWAYS_PORT},
        {"front distance sensor", DISTANCE_FRONT_PORT},    {"back distance sensor", DISTANCE_BACK_PORT},
        {"left distance sensor", DISTANCE_LEFT_PORT},      {"right distance sensor", DISTANCE_RIGHT_PORT},
    };
    int device_count = sizeof(devices) / sizeof(devices[0]);
    for (int i = 0; i < device_count; i++) {
        for (int j = i + 1; j < device_count; j++) {
            if (devices[i].port >= 0 && devices[i].port == devices[j].port) {
                report("Port %d is used twice: %s and %s", devices[i].port + 1, devices[i].name, devices[j].name);
                problems++;
            }
        }
    }

    for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
        DriveMotor &m = drive_motors[i];
        if (!m.device->installed()) {
            report("No motor on port %d (%s)", m.port + 1, m.name);
            problems++;
        } else {
            double temperature = m.device->temperature(temperatureUnits::celsius);
            if (temperature >= MOTOR_HOT_CELSIUS) {
                report("%s motor is hot (%.0f C): it loses power", m.name, temperature);
                problems++;
            }
        }
    }

    if (!Inertial.installed()) {
        report("No inertial sensor on port %d", PORT_INERTIAL + 1);
        problems++;
    }
    // (a second rotation object on the same port is fine, it just talks to the same sensor)
    if (TRACKING_FORWARD_PORT >= 0 && !rotation(TRACKING_FORWARD_PORT).installed()) {
        report("No rotation sensor on port %d (forward tracking wheel)", TRACKING_FORWARD_PORT + 1);
        problems++;
    }
    if (TRACKING_SIDEWAYS_PORT >= 0 && !rotation(TRACKING_SIDEWAYS_PORT).installed()) {
        report("No rotation sensor on port %d (sideways tracking wheel)", TRACKING_SIDEWAYS_PORT + 1);
        problems++;
    }
    for (int i = device_count - 4; i < device_count; i++) { // the distance sensors: the last four in the list above
        if (devices[i].port >= 0 && !distance(devices[i].port).installed()) {
            report("No distance sensor on port %d (%s)", devices[i].port + 1, devices[i].name);
            problems++;
        }
    }

    if (problems == 0) {
        report("Everything is plugged in.");
        controllerMessage("Devices OK", "");
    } else {
        char line[32];
        snprintf(line, sizeof(line), "%d PROBLEM%s", problems, problems == 1 ? "" : "S");
        controllerMessage(line, "see Brain screen");
        Controller.rumble("---");
    }
    return problems == 0;
}

// ============================================================================
// testDrivetrain
// ============================================================================

const double TEST_POWER = 30;  // percent
const double TEST_TIME_MS = 600;

// What we measured while one side of the drivetrain drove forward
struct SideTest {
    double motor_dps[DRIVE_MOTOR_COUNT]; // how fast each drive motor turned (degrees/s)
    double gyro_rate;                    // getGyroRate() while turning
    double turned;                       // how much the robot turned, in degrees (clockwise is positive)
};

static SideTest driveOneSide(bool left) {
    SideTest result = {};
    double start_heading = getInertial();
    if (left) {
        move(TEST_POWER, 0);
        rightDrive.stop(brakeType::hold);
    } else {
        move(0, TEST_POWER);
        leftDrive.stop(brakeType::hold);
    }
    vexDelay(TEST_TIME_MS / 2); // get up to speed

    // Average a few readings, taken while the robot is turning steadily
    const int SAMPLES = 5;
    for (int s = 0; s < SAMPLES; s++) {
        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            result.motor_dps[i] += drive_motors[i].device->velocity(velocityUnits::dps) / SAMPLES;
        }
        result.gyro_rate += getGyroRate() / SAMPLES;
        vexDelay(TEST_TIME_MS / 2 / SAMPLES);
    }
    stopDriving(brakeType::brake);
    vexDelay(500); // wait until it has stopped
    result.turned = getInertial() - start_heading;
    return result;
}

bool testDrivetrain() {
    cancelMovement();
    startReport("Drivetrain test");
    int problems = 0;
    bool turned_wrong_way[2] = {false, false};
    bool gyro_rate_flipped = false;

    for (int step = 0; step < 2; step++) {
        bool left = (step == 0);
        const char* side = left ? "left" : "right";
        SideTest test = driveOneSide(left);

        // How fast the motors on the powered side turned, on average
        double side_speed = 0;
        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            if (drive_motors[i].left == left) {
                side_speed += fabs(test.motor_dps[i]) / 3;
            }
        }
        if (side_speed < 20) {
            report("The %s side didn't move: are its motors plugged in?", side);
            problems++;
            continue;
        }

        for (int i = 0; i < DRIVE_MOTOR_COUNT; i++) {
            DriveMotor &m = drive_motors[i];
            double speed = test.motor_dps[i];
            if (m.left == left) {
                // Powered forward: it must turn forward, about as fast as the others
                if (speed < -0.25 * side_speed) {
                    report("%s spins backwards: change %s", m.name, m.setting);
                    problems++;
                } else if (speed < 0.25 * side_speed) {
                    report("%s motor doesn't turn (unplugged?)", m.name);
                    problems++;
                }
            } else if (fabs(speed) > 0.25 * side_speed) {
                // Not powered, but turned anyway: it is probably on the other side of the robot
                report("%s moved with the %s side: check its port", m.name, side);
                problems++;
            }
        }

        // The left side driving forward turns the robot clockwise (heading goes up), the right side counter-clockwise
        double expected = left ? 1 : -1;
        if (fabs(test.turned) < 3) {
            report("The robot hardly turned (%.1f deg): check the inertial sensor", test.turned);
            problems++;
            continue;
        }
        turned_wrong_way[step] = (test.turned * expected < 0);
        if (test.gyro_rate * test.turned < 0) {
            gyro_rate_flipped = true; // it turned one way, but the turning speed says the other way
        }
    }

    if (turned_wrong_way[0] && turned_wrong_way[1]) {
        report("Both sides turn the robot the wrong way. Either:");
        report("- left and right ports are swapped, or");
        report("- every motor's direction is wrong (forward drives back)");
        problems++;
    } else {
        for (int step = 0; step < 2; step++) {
            if (turned_wrong_way[step]) {
                const char* sides[2] = {"left", "right"};
                const char* settings[2] = {"LF, LM and LB_DIRECTION", "RF, RM and RB_DIRECTION"};
                report("The %s side drove backwards: change %s", sides[step], settings[step]);
                problems++;
            }
        }
    }
    if (gyro_rate_flipped) {
        report("getGyroRate() has the wrong sign: remove its minus sign");
        report("(in simpleV5lib.cpp), or turns will shake wildly");
        problems++;
    }

    if (problems == 0) {
        report("Drivetrain OK: motors, sides and inertial sensor agree.");
        controllerMessage("Drivetrain OK", "");
    } else {
        controllerMessage("DRIVETRAIN PROBLEM", "see Brain screen");
        Controller.rumble("---");
    }
    return problems == 0;
}

// ============================================================================
// measureTrackWidth
// ============================================================================

const double MEASURE_TURNS = 3;
const double MEASURE_POWER = 35;          // percent
const double MEASURE_TIMEOUT_MS = 20000;

TrackWidthResult measureTrackWidth() {
    TrackWidthResult result = {0, 0, 0};
    cancelMovement();
    startOdometry(); // sets up the tracking wheel sensors
    startReport("Measuring the track width");
    report("Spinning %.0f times...", MEASURE_TURNS);
    controllerMessage("Measuring...", "");

    double start_heading = getInertial();
    double start_left = motorInches(leftFront);
    double start_right = motorInches(rightFront);
    double start_forward = getForwardTrackingWheel();
    double start_sideways = getSidewaysTrackingWheel();

    double start_time = Brain.timer(timeUnits::msec);
    while (fabs(getInertial() - start_heading) < MEASURE_TURNS * 360) {
        double time = Brain.timer(timeUnits::msec) - start_time;
        if (time > MEASURE_TIMEOUT_MS) {
            break;
        }
        // Speed up gently, so the wheels don't slip
        double power = fmin(MEASURE_POWER, 10 + time / 500 * (MEASURE_POWER - 10));
        move(power, -power); // clockwise
        vexDelay(10);
    }
    stopDriving(brakeType::brake);
    vexDelay(500); // let it stop; the sensors keep counting while it does, which is fine

    // While spinning in place, every wheel rolls along a circle around the robot's center:
    // inches rolled = distance from the center * angle turned (in radians)
    double turned = (getInertial() - start_heading) * M_PI / 180;
    if (fabs(turned) < M_PI) {
        report("The robot turned only %.0f degrees: is the inertial sensor plugged in?", turned * 180 / M_PI);
        controllerMessage("MEASURING FAILED", "see Brain screen");
        return result;
    }
    double left = (motorInches(leftFront) - start_left) * drive_wheel_scale;
    double right = (motorInches(rightFront) - start_right) * drive_wheel_scale;
    // The left wheels rolled forward and the right ones backward, each half the track width from the center
    result.track_width = (left - right) / turned;

    report("Put these in simpleV5LibConfig.h:");
    report("TRACK_WIDTH_INCH = %.2f  (now %.2f)", result.track_width, TRACK_WIDTH_INCH);
    // Signs as in odometry.cpp: turning clockwise rolls a forward wheel on the right side backwards,
    // and a sideways wheel in front of the center to the right
    if (TRACKING_FORWARD_PORT >= 0) {
        result.forward_offset = -(getForwardTrackingWheel() - start_forward) * tracking_wheel_scale / turned;
        report("TRACKING_FORWARD_OFFSET = %.2f  (now %.2f)", result.forward_offset, TRACKING_FORWARD_OFFSET);
    }
    if (TRACKING_SIDEWAYS_PORT >= 0) {
        // (both tracking wheels are TRACKING_WHEEL_DIAMETER_INCH, so the same correction fits both)
        result.sideways_offset = (getSidewaysTrackingWheel() - start_sideways) * tracking_wheel_scale / turned;
        report("TRACKING_SIDEWAYS_OFFSET = %.2f  (now %.2f)", result.sideways_offset, TRACKING_SIDEWAYS_OFFSET);
    }
    char line[32];
    snprintf(line, sizeof(line), "Track %.2f in", result.track_width);
    controllerMessage(line, "see Brain screen");
    return result;
}

// ============================================================================
// measureWheelSize
// ============================================================================

WheelSizeResult measureWheelSize(double push_inches) {
    WheelSizeResult result = {0, 0};
    cancelMovement();
    startOdometry(); // sets up the tracking wheel sensors
    stopDriving(brakeType::coast); // let the wheels roll freely, so the robot can be pushed
    startReport("Measuring the wheel size");
    report("Line the robot up, then press A (or tap here).");
    controllerMessage("Line up the robot", "then press A");
    waitForPress();

    double start_drive = getPosition();
    double start_forward = getForwardTrackingWheel();
    report("Push it straight forward %.1f inches, then press A.", push_inches);
    char line[32];
    snprintf(line, sizeof(line), "Push %.1f in", push_inches);
    controllerMessage(line, "then press A");
    waitForPress();

    // The library thinks the robot went this far. If the wheels are really a bit bigger, it went
    // further than that: the real size is the size we use * how far it really went / how far we think
    double drive = getPosition() - start_drive;
    report("Put these in simpleV5LibConfig.h:");
    if (drive < 1) {
        report("The drive wheels counted %.1f in: push it forwards", drive);
        report("(backwards means the motor directions are wrong)");
    } else {
        result.wheel_diameter = WHEEL_DIAMETER_INCH * push_inches / drive;
        drive_wheel_scale = push_inches / drive;
        report("WHEEL_DIAMETER_INCH = %.3f  (now %.3f)", result.wheel_diameter, WHEEL_DIAMETER_INCH);
    }
    if (TRACKING_FORWARD_PORT >= 0) {
        double forward = getForwardTrackingWheel() - start_forward;
        if (forward < 1) {
            report("The tracking wheel counted %.1f in:", forward);
            report("backwards? change TRACKING_FORWARD_REVERSED");
        } else {
            result.tracking_wheel_diameter = TRACKING_WHEEL_DIAMETER_INCH * push_inches / forward;
            tracking_wheel_scale = push_inches / forward;
            report("TRACKING_WHEEL_DIAMETER_INCH = %.3f  (now %.3f)", result.tracking_wheel_diameter,
                   TRACKING_WHEEL_DIAMETER_INCH);
        }
    }
    controllerMessage("Done", "see Brain screen");
    return result;
}
