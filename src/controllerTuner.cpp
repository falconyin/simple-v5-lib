#include "simpleV5lib.h"
#include <cstdio>

// Tuning the PID gains from the controller (tuneWithController). It changes turnGains,
// forwardGains, ... (see simpleV5lib.h), which the movements use from their next start.

enum tuneMode { TUNE_TURN, TUNE_FORWARD, TUNE_SWING, TUNE_ARC };
const int MODE_COUNT = 4;
static const char* MODE_NAMES[MODE_COUNT] = {"TURN", "FORWARD", "SWING", "ARC"};
static const char* PART_NAMES[3] = {"kP", "kI", "kD"};

static PIDGains &gainsFor(int mode) {
    if (mode == TUNE_TURN) {
        return turnGains;
    } else if (mode == TUNE_FORWARD) {
        return forwardGains;
    } else if (mode == TUNE_SWING) {
        return swingGains;
    }
    return arcGains;
}

// part: 0 = kP, 1 = kI, 2 = kD
static double &partOf(PIDGains &gains, int part) {
    if (part == 0) {
        return gains.kp;
    } else if (part == 1) {
        return gains.ki;
    }
    return gains.kd;
}

// Round to 2 digits (3.52 -> 3.5, 0.0473 -> 0.047), so the values stay easy to read and type
static double roundTo2Digits(double value) {
    if (value <= 0) {
        return 0;
    }
    double digit = pow(10, floor(log10(value)) - 1); // what the second digit counts: 3.5 -> 0.1
    return round(value / digit) * digit;
}

// One press changes a value by about 10%. From 0 it starts at 0.01, and below that it becomes 0.
static double bigger(double value) {
    return (value < 0.01) ? 0.01 : roundTo2Digits(value * 1.1);
}

static double smaller(double value) {
    return (value <= 0.01) ? 0 : roundTo2Digits(value / 1.1);
}

// Turns a button into "presses": true once when it goes down. With repeat, also true again
// every 0.2 s while it is held down for longer than 0.5 s.
struct ButtonPress {
    bool repeat;
    bool was_down = false;
    double down_since = 0;
    double last_repeat = 0;

    ButtonPress(bool repeat) : repeat(repeat) {}

    bool update(bool down, double now_ms) {
        bool pressed = false;
        if (down && !was_down) {
            pressed = true;
            down_since = now_ms;
            last_repeat = now_ms;
        } else if (down && repeat && now_ms - down_since > 500 && now_ms - last_repeat >= 200) {
            pressed = true;
            last_repeat = now_ms;
        }
        was_down = down;
        return pressed;
    }
};

// The test movement for each mode. Every other try goes back, so the robot stays in its spot.
static void tryMovement(int mode, bool go_back) {
    double sign = go_back ? -1 : 1;
    double heading = getInertial();
    if (mode == TUNE_TURN) {
        PID_turn(heading + 90 * sign, 0.5, 0.2);
    } else if (mode == TUNE_FORWARD) {
        PID_forward(24 * sign, 0.3, 0.2);
    } else if (mode == TUNE_SWING) {
        PID_swing(heading + 90 * sign, LEFT_SIDE, 0.5, 0.2); // going back, the left side drives backwards
    } else {
        PID_arc(heading + 90 * sign, 24 * sign, 0.5, 0.2);   // going back, it backs up along the same circle
    }
}

// The controller screen has 3 lines of 19 letters. It only takes a new line about every 50 ms,
// so wait a little between lines, or some of them never show up.
static void showLine(int row, const char* text) {
    Controller.Screen.clearLine(row);
    Controller.Screen.setCursor(row, 1);
    Controller.Screen.print("%s", text);
    vexDelay(50);
}

static void showTuner(int mode, int part, bool have_result, const MovementResult &result) {
    char line[64]; // longer than the screen, the controller cuts it off
    PIDGains &gains = gainsFor(mode);
    snprintf(line, sizeof(line), "%s %s=%.3g", MODE_NAMES[mode], PART_NAMES[part], partOf(gains, part));
    showLine(1, line);
    snprintf(line, sizeof(line), "P%.3g I%.3g D%.3g", gains.kp, gains.ki, gains.kd);
    showLine(2, line);
    if (!have_result) {
        snprintf(line, sizeof(line), "A:try X:mode B:end");
    } else if (result.timed_out) {
        snprintf(line, sizeof(line), "TIMEOUT err %.2g", result.error);
    } else {
        // time, overshoot and error at the end. Short names, the line is only 19 letters long.
        snprintf(line, sizeof(line), "%.2fs ov%.2g e%.2g", result.time_ms / 1000, result.overshoot, fabs(result.error));
    }
    showLine(3, line);
}

// The final gains, as lines to copy into simpleV5LibConfig.h
static void showFinalGains() {
    printf("\n// Tuned gains, copy into simpleV5LibConfig.h:\n");
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Tuned gains for simpleV5LibConfig.h:");
    for (int mode = 0; mode < MODE_COUNT; mode++) {
        PIDGains &gains = gainsFor(mode);
        printf("const double %s_KP = %g;\n", MODE_NAMES[mode], gains.kp);
        printf("const double %s_KI = %g;\n", MODE_NAMES[mode], gains.ki);
        printf("const double %s_KD = %g;\n", MODE_NAMES[mode], gains.kd);
        Brain.Screen.setCursor(3 + mode, 1);
        Brain.Screen.print("%-8s kP %-7g kI %-7g kD %g", MODE_NAMES[mode], gains.kp, gains.ki, gains.kd);
    }
    fflush(stdout);
    Controller.Screen.clearScreen();
    showLine(1, "Done: gains are on");
    showLine(2, "the Brain screen");
}

void tuneWithController() {
    cancelMovement();
    int mode = TUNE_TURN;
    int part = 0;
    bool go_back[MODE_COUNT] = {false, false, false, false}; // the next try of that mode goes back
    bool have_result = false;
    MovementResult result = {0, 0, 0, false};
    ButtonPress next_mode(false), part_up(false), part_down(false), try_it(false), done(false);
    ButtonPress more(true), less(true);

    Controller.Screen.clearScreen();
    showTuner(mode, part, have_result, result);
    while (true) {
        double now = Brain.timer(timeUnits::msec);
        bool changed = false;

        if (next_mode.update(Controller.ButtonX.pressing(), now)) {
            mode = (mode + 1) % MODE_COUNT;
            have_result = false; // that result was for the other movement
            changed = true;
        }
        if (part_up.update(Controller.ButtonUp.pressing(), now)) {
            part = (part + 2) % 3; // one up, kP wraps around to kD
            changed = true;
        }
        if (part_down.update(Controller.ButtonDown.pressing(), now)) {
            part = (part + 1) % 3;
            changed = true;
        }
        if (more.update(Controller.ButtonRight.pressing(), now)) {
            double &value = partOf(gainsFor(mode), part);
            value = bigger(value);
            changed = true;
        }
        if (less.update(Controller.ButtonLeft.pressing(), now)) {
            double &value = partOf(gainsFor(mode), part);
            value = smaller(value);
            changed = true;
        }
        if (try_it.update(Controller.ButtonA.pressing(), now)) {
            showLine(3, "Moving...");
            tryMovement(mode, go_back[mode]);
            go_back[mode] = !go_back[mode];
            result = lastMovementResult();
            have_result = true;
            changed = true;
        }
        if (done.update(Controller.ButtonB.pressing(), now)) {
            break;
        }

        if (changed) {
            showTuner(mode, part, have_result, result);
        }
        vexDelay(20);
    }
    showFinalGains();
}
