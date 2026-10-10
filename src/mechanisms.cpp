#include "simpleV5lib.h"

// ============================================================================
// The background task
// One task keeps every Arm and Intake going: it moves the arms, watches the intakes for jams.
// Your code only leaves a command for it (moveTo, spin, ...), so it never has to wait.
// ============================================================================

const int MAX_MECHANISMS = 8; // of each kind

static Arm* arms[MAX_MECHANISMS];
static int arm_count = 0;
static Intake* intakes[MAX_MECHANISMS];
static int intake_count = 0;
static task* mechanism_task = nullptr;

static int mechanismLoop() {
    while (true) {
        for (int i = 0; i < arm_count; i++) {
            arms[i]->update();
        }
        for (int i = 0; i < intake_count; i++) {
            intakes[i]->update();
        }
        vexDelay(10);
    }
    return 0;
}

// Started by the first command, not when the Arm or Intake is made: those are made before main()
// runs, too early to start a task
static void startMechanismTask() {
    if (mechanism_task == nullptr) {
        // Made with "new" so the task object is never destroyed, which keeps the task running
        mechanism_task = new task(mechanismLoop);
    }
}

// Power in percent (-100 to 100). 100% = 12 V = 12000 mV.
static void spinMotors(motor_group &motors, double power) {
    motors.spin(directionType::fwd, power * 120, voltageUnits::mV);
}

static double limitPower(double power, double max_power) {
    return fmax(-max_power, fmin(max_power, power));
}

// ============================================================================
// Arm
// ============================================================================

// What your code asked for
enum { ARM_NO_COMMAND, ARM_MOVE, ARM_MANUAL, ARM_HOLD_HERE, ARM_RELEASE };
// What the background task is doing
enum { ARM_LOOSE, ARM_MOVING, ARM_HOLDING, ARM_DRIVEN };

Arm::Arm(motor &one_motor, double kp, double ki, double kd)
    : own_motors(one_motor), motors(&own_motors), pid(kp, ki, kd, ARM_INTEGRAL_RANGE) {
    init();
}

Arm::Arm(motor_group &group, double kp, double ki, double kd)
    : motors(&group), pid(kp, ki, kd, ARM_INTEGRAL_RANGE) {
    init();
}

void Arm::init() {
    lowest = -1e9; // no limits until setLimits
    highest = 1e9;
    request = 0;
    command = ARM_NO_COMMAND;
    target = 0;
    max_speed = 100;
    manual_power = 0;
    finished_request = 0;
    arrived = true;
    last_command = ARM_NO_COMMAND;
    state = ARM_LOOSE;
    registered = arm_count < MAX_MECHANISMS;
    if (registered) {
        arms[arm_count] = this;
        arm_count++;
    }
}

// Leave a command for the background task
void Arm::send(int new_command) {
    if (!registered) {
        // The background task doesn't know this Arm, so nothing would ever happen. Say so, and make
        // waitUntilDone return false right away instead of waiting forever.
        printf("Only %d Arms can be used: this one does nothing\n", MAX_MECHANISMS);
        arrived = false;
        return;
    }
    last_command = new_command;
    command = new_command;
    request++; // the background task sees this change and picks up the command
    startMechanismTask();
}

void Arm::moveTo(double degrees, double speed) {
    degrees = fmax(lowest, fmin(highest, degrees));
    speed = fmin(100, fabs(speed)); // a speed limit, the direction comes from the PID
    if (last_command == ARM_MOVE && degrees == target && speed == max_speed) {
        return; // already on its way there, or holding there
    }
    target = degrees;
    max_speed = speed;
    send(ARM_MOVE);
}

bool Arm::isDone() {
    return finished_request == request;
}

bool Arm::waitUntilDone() {
    while (!isDone()) {
        vexDelay(10);
    }
    return arrived;
}

void Arm::manual(double power) {
    if (fabs(power) < DRIVE_DEADBAND) {
        power = 0; // a joystick that doesn't sit exactly at 0 must still let the arm hold
    }
    if (power != 0) {
        manual_power = power;
        if (last_command != ARM_MANUAL) {
            send(ARM_MANUAL);
        }
    } else if (last_command == ARM_MANUAL) {
        send(ARM_HOLD_HERE); // just let go of the stick: stay right here
    }
    // 0 after a moveTo: let the moveTo finish
}

void Arm::setLimits(double low, double high) {
    lowest = low;
    highest = high;
}

void Arm::resetPosition(double degrees) {
    motors->setPosition(degrees, rotationUnits::deg);
}

double Arm::position() {
    return motors->position(rotationUnits::deg);
}

void Arm::release() {
    send(ARM_RELEASE);
}

// Start doing what the newest command asks
void Arm::begin(int new_command) {
    if (new_command == ARM_MOVE) {
        pid.reset(target - position());
        start_time = Brain.timer(msec);
        settle_start = -1;
        state = ARM_MOVING;
    } else if (new_command == ARM_MANUAL) {
        held_at_limit = 0;
        state = ARM_DRIVEN;
    } else if (new_command == ARM_HOLD_HERE) {
        motors->stop(brakeType::hold);
        state = ARM_HOLDING;
    } else if (new_command == ARM_RELEASE) {
        motors->stop(brakeType::coast);
        state = ARM_LOOSE;
    }
    if (new_command != ARM_MOVE) {
        arrived = true;
        finished_request = seen_request; // nothing to wait for
    }
}

void Arm::update() {
    int newest = request;
    if (newest != seen_request) {
        seen_request = newest;
        begin(command);
    }
    double now = Brain.timer(msec);
    double pos = position();

    if (state == ARM_MOVING) {
        double error = target - pos;
        // Close enough for long enough, so it isn't just swinging through the target
        if (fabs(error) < ARM_TOLERANCE) {
            if (settle_start < 0) {
                settle_start = now;
            }
        } else {
            settle_start = -1;
        }
        bool there = settle_start >= 0 && now - settle_start >= ARM_SETTLE_MS;
        bool gave_up = now - start_time >= ARM_TIMEOUT_MS;
        if (there || gave_up) {
            // The motor's own hold mode keeps it here from now on, even with something heavy on it
            motors->stop(brakeType::hold);
            state = ARM_HOLDING;
            arrived = there; // first, so it is ready once finished_request says this move is done
            finished_request = seen_request;
            return;
        }
        double rate = motors->velocity(velocityUnits::dps) / 100; // degrees per 10 ms
        spinMotors(*motors, limitPower(pid.compute(error, rate), max_speed));
    } else if (state == ARM_DRIVEN) {
        double power = manual_power;
        int pushing = (power > 0) ? 1 : -1;
        if (held_at_limit == pushing) {
            // Still pushing into the limit: keep holding. Even if the arm sags back a little, or it
            // would drive up, hold, sag, drive up, ...
        } else if ((pushing == 1 && pos >= highest) || (pushing == -1 && pos <= lowest)) {
            motors->stop(brakeType::hold);
            held_at_limit = pushing;
        } else {
            held_at_limit = 0;
            spinMotors(*motors, power);
        }
    }
}

// ============================================================================
// Intake
// ============================================================================

// What the background task is doing
enum { INTAKE_STOPPED, INTAKE_RUNNING, INTAKE_UNJAMMING, INTAKE_STOPPED_BY_JAM };

Intake::Intake(motor &one_motor) : own_motors(one_motor), motors(&own_motors) {
    init();
}

Intake::Intake(motor_group &group) : motors(&group) {
    init();
}

void Intake::init() {
    request = 0;
    power = 0;
    unjam = true;
    jammed = false;
    jams = 0;
    last_power = 0;
    last_unjam = true;
    state = INTAKE_STOPPED;
    registered = intake_count < MAX_MECHANISMS;
    if (registered) {
        intakes[intake_count] = this;
        intake_count++;
    }
}

void Intake::spin(double new_power, bool new_unjam) {
    if (!registered) {
        printf("Only %d Intakes can be used: this one does nothing\n", MAX_MECHANISMS);
        return;
    }
    if (new_power == last_power && new_unjam == last_unjam) {
        return; // nothing new: don't start over (a jammed intake stays stopped)
    }
    last_power = new_power;
    last_unjam = new_unjam;
    power = new_power;
    unjam = new_unjam;
    request++; // the background task sees this change and picks up the command
    startMechanismTask();
}

void Intake::stop() {
    spin(0, last_unjam);
}

bool Intake::isJammed() {
    return jammed;
}

int Intake::jamCount() {
    return jams;
}

void Intake::update() {
    double now = Brain.timer(msec);
    int newest = request;
    if (newest != seen_request) {
        seen_request = newest;
        running_power = power;
        jammed = false;
        jam_start = -1;
        if (running_power == 0) {
            motors->stop(brakeType::coast);
            state = INTAKE_STOPPED;
        } else {
            spinMotors(*motors, running_power);
            state = INTAKE_RUNNING;
        }
    }

    if (state == INTAKE_RUNNING) {
        // A jam: told to spin, drawing a lot of current, but hardly turning. It must last
        // INTAKE_JAM_MS, because starting up looks the same for a moment.
        double amps = motors->current(currentUnits::amp) / motors->count(); // current() is all motors together
        double speed = fabs(motors->velocity(velocityUnits::pct));
        if (amps > INTAKE_JAM_CURRENT && speed < INTAKE_JAM_SPEED) {
            if (jam_start < 0) {
                jam_start = now;
            }
        } else {
            jam_start = -1;
        }
        if (jam_start >= 0 && now - jam_start >= INTAKE_JAM_MS) {
            jams++;
            jammed = true;
            jam_start = -1;
            if (unjam) {
                spinMotors(*motors, -running_power); // back off, to let the stuck piece go
                state = INTAKE_UNJAMMING;
                unjam_start = now;
            } else {
                motors->stop(brakeType::coast);
                state = INTAKE_STOPPED_BY_JAM;
            }
        }
    } else if (state == INTAKE_UNJAMMING && now - unjam_start >= INTAKE_UNJAM_MS) {
        spinMotors(*motors, running_power); // try again
        jammed = false;
        state = INTAKE_RUNNING;
    }
}
