// Tests for simple-v5-lib, run on a computer against the simulator in tests/sim/vex.h.
// Build and run them with tests/run_tests.sh. Every check prints PASS or FAIL;
// the program exits with the number of failures, so 0 means everything passed.

#include "simpleV5lib.h"
#include <cstdio>
#include <unistd.h>
#include <string>
#include <vector>
static int auton_runs = 0;
static void testAuton() { auton_runs++; }
static int fails = 0;
static void check(const char* name, bool ok, double a, double b) {
    printf("%-46s %s  (%.2f, %.2f)\n", name, ok ? "PASS" : "FAIL", a, b);
    if (!ok) fails++;
}
static void settle() { stopDriving(); vexDelay(500); }
// How far odometry's position is from where the simulated robot really is
static double odometryError() { return hypot(getX() - sim::x, getY() - sim::y); }

// Mechanisms, made outside any function like in a robot program. The simulator treats ports 7-21
// as mechanism motors; sim:: arrays are indexed by port index (PORT15 = 14).
static motor armMotor(PORT15, ratio36_1, false);
static Arm arm(armMotor);
static motor liftLeft(PORT16, ratio36_1, false);
static motor liftRight(PORT17, ratio36_1, false);
static motor_group liftMotors(liftLeft, liftRight);
static Arm lift(liftMotors);
static motor intakeMotor(PORT18, ratio6_1, false);
static Intake intake(intakeMotor);
static motor rollerA(PORT19, ratio6_1, false);
static motor rollerB(PORT20, ratio6_1, false);
static motor_group rollerMotors(rollerA, rollerB);
static Intake roller(rollerMotors);
const int ARM = PORT15, INTAKE = PORT18;
// Call arm.manual(power) every 20 ms for this long, like a driver control loop
static void driveArm(double power, double ms) {
    for (double t = 0; t < ms; t += 20) { arm.manual(power); vexDelay(20); }
}

int main() {
    // Tell the simulator where the tracking wheels from simpleV5LibConfig.h are (if any)
    sim::fwd_wheel_port = TRACKING_FORWARD_PORT;
    sim::fwd_wheel_offset = TRACKING_FORWARD_OFFSET;
    sim::side_wheel_port = TRACKING_SIDEWAYS_PORT;
    sim::side_wheel_offset = TRACKING_SIDEWAYS_OFFSET;
    sim::track_wheel_diam = TRACKING_WHEEL_DIAMETER_INCH;
    printf("tracking wheels: forward port %d, sideways port %d\n", TRACKING_FORWARD_PORT, TRACKING_SIDEWAYS_PORT);
    // ... and the distance sensors
    struct { int port; double ahead, right, looks; } distance_sensors[] = {
        {DISTANCE_FRONT_PORT, DISTANCE_FRONT_AHEAD, DISTANCE_FRONT_RIGHT, 0},
        {DISTANCE_BACK_PORT, DISTANCE_BACK_AHEAD, DISTANCE_BACK_RIGHT, 180},
        {DISTANCE_LEFT_PORT, DISTANCE_LEFT_AHEAD, DISTANCE_LEFT_RIGHT, -90},
        {DISTANCE_RIGHT_PORT, DISTANCE_RIGHT_AHEAD, DISTANCE_RIGHT_RIGHT, 90},
    };
    for (auto &s : distance_sensors) {
        if (s.port >= 0) sim::distance_mount[s.port] = {true, s.ahead, s.right, s.looks};
    }

    double t0, d0;
    // forward
    t0 = sim::t_ms; d0 = getPosition();
    PID_forward(48, 0.3, 0.2);
    settle();
    check("forward 48in: ends near 48, before timeout", fabs(getPosition() - d0 - 48) < 1 && sim::t_ms - t0 < 5000, getPosition() - d0, sim::t_ms - t0);
    // backward
    t0 = sim::t_ms; d0 = getPosition();
    PID_forward(-24, 0.3, 0.2);
    settle();
    check("backward 24in", fabs(getPosition() - d0 + 24) < 1 && sim::t_ms - t0 < 5000, getPosition() - d0, sim::t_ms - t0);
    // max speed
    sim::peak = 0;
    PID_forward(48, 0.3, 0.2, FORWARD_TIMEOUT_MS, 40);
    check("max_speed 40%: peak wheel speed <= 40% of 70in/s", sim::peak <= 0.40 * 70 + 0.5, sim::peak, 28);
    settle();
    // heading hold: right side 15% weaker, so the robot pulls clockwise
    sim::gain[1] = 0.85;
    setHeading(0);
    PID_forward(72, 0.3, 0.2); settle();
    double with_hold = getInertial();
    check("forward with weak right side keeps heading", fabs(with_hold) < 2, with_hold, 0);
    sim::gain[1] = 1;
    // turns
    setHeading(0);
    t0 = sim::t_ms; PID_turn(90, 0.5, 0.2); settle();
    check("turn to 90", fabs(getInertial() - 90) < 1.5 && sim::t_ms - t0 < 3000, getInertial(), sim::t_ms - t0);
    t0 = sim::t_ms; PID_turn(-45, 0.5, 0.2); settle();
    check("turn to -45", fabs(getInertial() + 45) < 1.5 && sim::t_ms - t0 < 3000, getInertial(), sim::t_ms - t0);
    PID_turn_relative(30, 0.5, 0.2); settle();
    check("turn relative +30 -> -15", fabs(getInertial() + 15) < 1.5, getInertial(), -15);
    setHeading(350);
    PID_turn_shortest(10, 0.5, 0.2); settle();
    check("shortest 350 -> 10 goes clockwise to 370", fabs(getInertial() - 370) < 1.5, getInertial(), 370);
    setHeading(10);
    PID_turn_shortest(350, 0.5, 0.2); settle();
    check("shortest 10 -> 350 goes ccw to -10", fabs(getInertial() + 10) < 1.5, getInertial(), -10);
    // swings
    setHeading(0); double l0 = sim::dist[0], r0 = sim::dist[1];
    t0 = sim::t_ms; PID_swing(90, LEFT_SIDE, 0.5, 0.2); settle();
    check("swing left side to 90", fabs(getInertial() - 90) < 1.5 && sim::t_ms - t0 < 3000, getInertial(), sim::t_ms - t0);
    check("  right side barely moved", fabs(sim::dist[1] - r0) < 0.5, sim::dist[1] - r0, sim::dist[0] - l0);
    l0 = sim::dist[0]; r0 = sim::dist[1];
    t0 = sim::t_ms; PID_swing(45, RIGHT_SIDE, 0.5, 0.2); settle();
    check("swing right side to 45", fabs(getInertial() - 45) < 1.5 && sim::t_ms - t0 < 3000, getInertial(), sim::t_ms - t0);
    check("  left side barely moved", fabs(sim::dist[0] - l0) < 0.5, sim::dist[0] - l0, sim::dist[1] - r0);
    // timeout: impossible tolerance
    t0 = sim::t_ms; PID_turn(0, 0.0, 0.0, 800); 
    check("timeout stops impossible turn at ~800ms", sim::t_ms - t0 < 900, sim::t_ms - t0, 800);
    // driver control
    Controller.Axis3.value = 50; Controller.Axis1.value = 0; arcadeDrive();
    check("arcade half stick -> 25% both sides (curve 2)", fabs(sim::cmd[0] - 25) < 0.5 && fabs(sim::cmd[1] - 25) < 0.5, sim::cmd[0], sim::cmd[1]);
    Controller.Axis3.value = 3; arcadeDrive();
    check("deadband: stick 3 -> 0", sim::cmd[0] == 0 && sim::cmd[1] == 0, sim::cmd[0], sim::cmd[1]);
    Controller.Axis3.value = 100; Controller.Axis1.value = 100; arcadeDrive();
    check("full fwd + full turn -> scaled 100/0", fabs(sim::cmd[0] - 100) < 0.5 && fabs(sim::cmd[1]) < 0.5, sim::cmd[0], sim::cmd[1]);
    Controller.Axis3.value = -100; Controller.Axis2.value = 100; tankDrive();
    check("tank -100/100", sim::cmd[0] == -100 && sim::cmd[1] == 100, sim::cmd[0], sim::cmd[1]);
    Controller.Axis3.value = 0; Controller.Axis1.value = 0; Controller.Axis2.value = 0;
    settle();

    // ---------- async ----------
    double t1 = sim::t_ms; d0 = getPosition();
    PID_forward_async(48, 0.3, 0.2);
    check("async returns right away, isMoving", sim::t_ms - t1 < 1 && isMoving(), sim::t_ms - t1, isMoving());
    waitUntilTraveled(12);
    double at = getPosition() - d0;
    check("waitUntilTraveled(12) returns at ~12in, still moving", at >= 12 && at < 14 && isMoving(), at, isMoving());
    waitUntilDone();
    check("waitUntilDone: arrived at 48, not moving", fabs(getPosition() - d0 - 48) < 1 && !isMoving(), getPosition() - d0, isMoving());
    settle();

    // queued movements: second waits for the first
    d0 = getPosition();
    PID_forward_async(24, 0.3, 0.2);
    PID_forward_async(-12, 0.3, 0.2); // must wait for the first
    waitUntilDone(); settle();
    check("two async moves run one after another: 24 then -12", fabs(getPosition() - d0 - 12) < 1, getPosition() - d0, 12);

    // relative async turn measured after previous movement
    setHeading(0);
    PID_turn_async(90, 0.5, 0.2);
    PID_turn_relative_async(30, 0.5, 0.2);
    waitUntilDone(); settle();
    check("relative async turn starts from the finished heading", fabs(getInertial() - 120) < 1.5, getInertial(), 120);

    // swing async + waitUntilTraveled in degrees
    setHeading(0);
    PID_swing_async(90, LEFT_SIDE, 0.5, 0.2);
    waitUntilTraveled(45);
    check("waitUntilTraveled(45) during swing", getInertial() >= 45 && getInertial() < 55 && isMoving(), getInertial(), isMoving());
    waitUntilDone(); settle();

    // cancel
    d0 = getPosition();
    PID_forward_async(72, 0.3, 0.2);
    waitUntilTraveled(10);
    t1 = sim::t_ms;
    cancelMovement();
    check("cancelMovement stops quickly", !isMoving() && sim::t_ms - t1 <= 15, sim::t_ms - t1, isMoving());
    settle();
    check("  robot stopped well short of 72", getPosition() - d0 < 30, getPosition() - d0, 72);
    cancelMovement(); // nothing running: must not hang
    check("cancelMovement with nothing running returns", true, 0, 0);

    // driver control takes over
    PID_forward_async(72, 0.3, 0.2);
    waitUntilTraveled(5);
    arcadeDrive();
    check("arcadeDrive cancels the running movement", !isMoving() && sim::cmd[0] == 0, isMoving(), sim::cmd[0]);
    settle();

    // the next movement works after a cancel
    d0 = getPosition();
    PID_forward(24, 0.3, 0.2); settle();
    check("movement after cancel works", fabs(getPosition() - d0 - 24) < 1, getPosition() - d0, 24);

    // ---------- telemetry ----------
    logToTerminal(true); graphOnScreen(true);
    setHeading(0);
    PID_turn(90, 0.5, 0.2);
    logToTerminal(false);
    check("screen graph drew lines", Brain.Screen.lines > 50 && Brain.Screen.clears == 1, Brain.Screen.lines, Brain.Screen.clears);
    PID_forward(-24, 0.3, 0.2); // graph only
    graphOnScreen(false);
    check("graph cleared for the next movement", Brain.Screen.clears == 2, Brain.Screen.clears, 2);

    // ---------- arcs ----------
    auto reset = []{ settle(); sim::x = 0; sim::y = 0; setPose(0, 0, 0); };
    reset();
    t1 = sim::t_ms;
    PID_arc(90, 24, 0.5, 0.2); settle();
    printf("   arc R24 right: x=%.2f y=%.2f h=%.2f t=%.0f\n", sim::x, sim::y, getInertial(), sim::t_ms - t1);
    check("arc forward right R24 -> heading 90 at (24,24)", fabs(getInertial() - 90) < 1.5 && hypot(sim::x - 24, sim::y - 24) < 1.5, sim::x, sim::y);
    reset();
    PID_arc(-90, 24, 0.5, 0.2); settle();
    check("arc forward left R24 -> heading -90 at (-24,24)", fabs(getInertial() + 90) < 1.5 && hypot(sim::x + 24, sim::y - 24) < 1.5, sim::x, sim::y);
    reset();
    PID_arc(90, -24, 0.5, 0.2); settle();
    check("arc backward R24 -> heading 90 at (-24,-24)", fabs(getInertial() - 90) < 1.5 && hypot(sim::x + 24, sim::y + 24) < 1.5, sim::x, sim::y);
    reset();
    PID_arc(45, 48, 0.5, 0.2); settle();
    check("arc R48 to 45 -> (48(1-cos45), 48 sin45)", fabs(getInertial() - 45) < 1.5 && hypot(sim::x - 48 * (1 - cos(M_PI / 4)), sim::y - 48 * sin(M_PI / 4)) < 1.5, sim::x, sim::y);
    reset(); sim::peak = 0;
    PID_arc(90, 24, 0.5, 0.2, FORWARD_TIMEOUT_MS, 50); settle();
    check("arc max_speed 50%: outside wheels <= 35in/s", sim::peak <= 35.5 && fabs(getInertial() - 90) < 1.5, sim::peak, 35);
    reset();
    PID_arc(90, 0, 0.5, 0.2); settle();
    check("arc radius 0 = turn in place", fabs(getInertial() - 90) < 1.5 && hypot(sim::x, sim::y) < 0.5, getInertial(), hypot(sim::x, sim::y));

    // ---------- chaining ----------
    reset(); t1 = sim::t_ms;
    PID_forward(24, 0.3, 0.2); PID_turn(90, 0.5, 0.2); PID_forward(24, 0.3, 0.2);
    double normal_time = sim::t_ms - t1;
    reset(); t1 = sim::t_ms;
    PID_forward_chain(24, 3);
    double speed_at_handover = getMotorRate();
    PID_turn_chain(90, 10);
    PID_forward(24, 0.3, 0.2);
    double chain_time = sim::t_ms - t1;
    printf("   normal %.0f ms, chained %.0f ms, end x=%.2f y=%.2f h=%.2f\n", normal_time, chain_time, sim::x, sim::y, getInertial());
    check("chained route is much faster than stopping each time", chain_time < normal_time * 0.8, chain_time, normal_time);
    check("  still moving fast when the chained drive hands over", speed_at_handover > 20, speed_at_handover, 20);
    check("  ends facing 90 (forward kept the turn's target)", fabs(getInertial() - 90) < 1.5, getInertial(), 90);
    check("  ends near (24,24)", hypot(sim::x - 24, sim::y - 24) < 2, sim::x, sim::y);

    reset();
    PID_turn_chain(90, 10);
    PID_turn_relative(90, 0.5, 0.2); settle();
    check("relative turn after chained turn counts from 90 -> 180", fabs(getInertial() - 180) < 1.5, getInertial(), 180);

    reset();
    PID_forward_chain(-24, 3);
    PID_forward(-12, 0.3, 0.2); settle();
    check("backward chain -24 then -12 -> -36", fabs(sim::y + 36) < 1, sim::y, -36);

    reset();
    PID_forward_chain(24, 3);
    PID_forward_chain(24, 3);
    PID_forward(24, 0.3, 0.2); settle();
    check("three chained forwards 24+24+24 -> 72", fabs(sim::y - 72) < 1, sim::y, 72);

    reset();
    PID_arc_chain(90, 24, 10);
    PID_forward(12, 0.3, 0.2); settle();
    check("chained arc then forward: faces 90", fabs(getInertial() - 90) < 1.5, getInertial(), 90);

    // chain as the last movement: must stop on its own
    reset();
    PID_forward_chain(24, 3);
    vexDelay(400);
    check("chain with nothing after it stops the robot", fabs(getMotorRate()) < 1 && sim::mode[0] == 2, getMotorRate(), sim::mode[0]);

    // driver takes over right after a chain: no surprise stop later
    reset();
    PID_forward_chain(24, 3);
    Controller.Axis3.value = 50; arcadeDrive();
    vexDelay(300); arcadeDrive();
    check("driver after chain: idle stop doesn't fire during driving", sim::mode[0] == 0 && fabs(sim::cmd[0] - 25) < 0.5, sim::mode[0], sim::cmd[0]);
    Controller.Axis3.value = 0; arcadeDrive();

    // cancel right after a chain stops the motors
    reset();
    PID_forward_chain(24, 3);
    cancelMovement();
    check("cancelMovement after a chain stops the motors", sim::mode[0] == 2 && sim::mode[1] == 2, sim::mode[0], sim::mode[1]);

    // ---------- review fixes ----------
    reset(); sim::gain[1] = 0.7; sim::peak_cmd = 0;
    PID_forward(48, 0.3, 0.2, FORWARD_TIMEOUT_MS, 50); settle();
    sim::gain[1] = 1;
    check("max_speed 50 with heading correction: no side above 50%", sim::peak_cmd <= 50.01, sim::peak_cmd, 50);
    reset(); sim::peak_cmd = 0;
    PID_arc(90, 6, 0.5, 0.2, FORWARD_TIMEOUT_MS, 30); settle();
    check("tight arc max_speed 30: no side above 30%", sim::peak_cmd <= 30.01 && fabs(getInertial() - 90) < 1.5, sim::peak_cmd, getInertial());

    int tasks_before = sim::next_id;
    addAuton("A", testAuton); addAuton("B", testAuton);
    startAutonSelector(); vexDelay(30);
    stopAutonSelector();
    startAutonSelector(); vexDelay(100);
    stopAutonSelector(); startAutonSelector(); vexDelay(100);
    check("selector stop/start never makes a second task", sim::next_id - tasks_before == 1, sim::next_id - tasks_before, 1);
    runSelectedAuton();
    check("runSelectedAuton runs the routine", auton_runs == 1, auton_runs, 1);

    // ---------- odometry ----------
    reset();
    PID_forward(48, 0.3, 0.2); PID_turn(90, 0.5, 0.2); PID_arc(180, 24, 0.5, 0.2);
    PID_swing(90, LEFT_SIDE, 0.5, 0.2); PID_forward(-24, 0.3, 0.2); settle();
    printf("   odometry (%.2f, %.2f), real (%.2f, %.2f)\n", getX(), getY(), sim::x, sim::y);
    check("odometry follows drives, turns, an arc and a swing", odometryError() < 0.5, odometryError(), 0.5);

    reset(); sim::gain[1] = 0.6;
    move(60, 60); vexDelay(1500); settle(); // weak right side: the robot drives a curve
    sim::gain[1] = 1;
    check("odometry follows a curving drive", odometryError() < 0.5 && hypot(sim::x, sim::y) > 30, odometryError(), hypot(sim::x, sim::y));

    settle(); sim::x = 10; sim::y = 20; setPose(10, 20, 90);
    PID_forward(12, 0.3, 0.2); settle();
    check("setPose(10, 20, 90) then forward 12 -> (22, 20)", fabs(getX() - 22) < 0.5 && fabs(getY() - 20) < 0.5, getX(), getY());

    reset();
    PID_turn_to_point(24, 24, 0.5, 0.2); settle();
    check("turn to point (24, 24) -> facing 45", fabs(getInertial() - 45) < 1.5, getInertial(), 45);
    reset();
    PID_turn_to_point(0, 24, 0.5, 0.2, TURN_TIMEOUT_MS, 100, true); settle();
    check("turn to point (0, 24) backwards -> facing 180", fabs(fabs(getInertial()) - 180) < 1.5, getInertial(), 180);

    reset();
    PID_drive_to_point(24, 24, 0.5, 0.2); settle();
    check("drive to point (24, 24)", hypot(sim::x - 24, sim::y - 24) < 1, sim::x, sim::y);
    reset();
    PID_drive_to_point(0, -24, 0.5, 0.2); settle();
    check("drive to point behind (0, -24): turns around first", hypot(sim::x, sim::y + 24) < 1, sim::x, sim::y);
    reset();
    PID_drive_to_point(0, -24, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, true); settle();
    check("drive to (0, -24) backwards: no turning", hypot(sim::x, sim::y + 24) < 1 && fabs(getInertial()) < 3, sim::y, getInertial());
    reset(); sim::gain[1] = 0.8;
    PID_drive_to_point(12, 48, 0.5, 0.2); settle();
    sim::gain[1] = 1;
    check("drive to point (12, 48) with a weak right side", hypot(sim::x - 12, sim::y - 48) < 1, sim::x, sim::y);

    reset();
    PID_drive_to_point(3, 0, 1, 0.2); settle(); // close, and straight to the side: must turn, then drive
    check("drive to a close point beside the robot (3, 0)", hypot(sim::x - 3, sim::y) < 1, sim::x, sim::y);

    reset();
    PID_drive_to_point(0, 24, 0.5, 0.2); PID_drive_to_point(24, 24, 0.5, 0.2);
    PID_drive_to_point(24, 0, 0.5, 0.2); PID_drive_to_point(0, 0, 0.5, 0.2); settle();
    check("drive a 24 inch square back to the start", hypot(sim::x, sim::y) < 1.5, sim::x, sim::y);

    reset();
    PID_drive_to_point_async(0, 48, 0.5, 0.2);
    waitUntilTraveled(24);
    check("waitUntilTraveled(24) during drive to point", sim::y >= 24 && sim::y < 28 && isMoving(), sim::y, isMoving());
    waitUntilDone();
    reset();
    PID_drive_to_point_async(48, 0, 0.5, 0.2); // turns 90 degrees first
    waitUntilTraveled(12);
    check("  counts inches, not the degrees of the first turn", hypot(sim::x, sim::y) >= 12, hypot(sim::x, sim::y), 12);
    waitUntilDone();

    // ---------- Chained drive to point ----------
    // A path with three 90 degree corners
    reset(); t1 = sim::t_ms;
    PID_drive_to_point(0, 24, 0.5, 0.2); PID_drive_to_point(24, 24, 0.5, 0.2);
    PID_drive_to_point(24, 48, 0.5, 0.2); PID_drive_to_point(48, 48, 0.5, 0.2);
    double normal_points = sim::t_ms - t1;
    reset(); t1 = sim::t_ms;
    PID_drive_to_point_chain(0, 24, 4);
    double point_handover_speed = fabs(getMotorRate());
    // From here until the last chained point, keep track of the slowest the robot drives
    static bool watch_speed = true;
    static double slowest = 1000;
    task speed_watch([]() -> int {
        while (watch_speed) { slowest = fmin(slowest, fabs(sim::v[0] + sim::v[1]) / 2); vexDelay(5); }
        return 0;
    });
    PID_drive_to_point_chain(24, 24, 4);
    PID_drive_to_point_chain(24, 48, 4);
    watch_speed = false;
    PID_drive_to_point(48, 48, 0.5, 0.2);
    double chained_points = sim::t_ms - t1;
    settle();
    printf("   points: normal %.0f ms, chained %.0f ms, handover %.1f in/s, slowest at the corners %.1f in/s, end (%.2f, %.2f)\n",
           normal_points, chained_points, point_handover_speed, slowest, sim::x, sim::y);
    check("chained points are much faster than stopping at each", chained_points < normal_points * 0.8, chained_points, normal_points);
    // (aiming past the point keeps it near full speed: about 55 in/s here, about 35 without)
    check("  still moving fast at the handover", point_handover_speed > 45, point_handover_speed, 45);
    check("  curves around the corners, never stops", slowest > 10, slowest, 10);
    check("  ends at the last point (48, 48)", hypot(sim::x - 48, sim::y - 48) < 0.5, sim::x, sim::y);
    reset();
    PID_drive_to_point_chain(0, 24, 4);
    PID_drive_to_point_chain(24, 48, 4);
    PID_drive_to_point(48, 48, 0.5, 0.2); // the last, normal movement turns first and drives straight in
    settle();
    check("chain then a 45 degree corner: stops exactly at the point", hypot(sim::x - 48, sim::y - 48) < 0.5, sim::x, sim::y);
    reset();
    PID_drive_to_point_chain(0, 24, 4);
    vexDelay(50);
    double rolling_speed = fabs(getMotorRate());
    vexDelay(CHAIN_STOP_AFTER_MS + 300);
    check("point chain keeps driving after it hands over", rolling_speed > 40, rolling_speed, 40);
    check("  with nothing after it, the robot stops", fabs(getMotorRate()) < 1 && fabs(sim::y - 24) < 6, getMotorRate(), sim::y);
    reset();
    PID_drive_to_point_chain(4, 0, 1); // close beside the robot (no turning first): drive there, don't end at once
    vexDelay(CHAIN_STOP_AFTER_MS + 300);
    check("point close beside the robot isn't skipped by a chain", sim::x > 2, sim::x, 2);
    reset();
    PID_drive_to_point_chain(0, -24, 4, FORWARD_TIMEOUT_MS, 100, true);
    double backed_up = sim::y;
    PID_drive_to_point(0, -48, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, true);
    check("backwards point chain hands over near (0, -24)", backed_up < -18 && backed_up > -24, backed_up, -20);
    check("  then -> (0, -48), not turned around", hypot(sim::x, sim::y + 48) < 1 && fabs(getInertial()) < 5, sim::y, getInertial());
    reset();
    PID_drive_to_point_chain(0, 24, 4);
    PID_drive_to_point(0, -12, 0.5, 0.2); // behind the robot: too far around to curve, turns first
    check("chain then a point behind: turns around and gets there", hypot(sim::x, sim::y + 12) < 1, sim::x, sim::y);
    reset();
    PID_drive_to_point_chain(0, 24, 4);
    // The robot is still rolling, so it aims at the point from where it is when the turn starts
    double aim_to = atan2(24 - getX(), 48 - getY()) * 180 / M_PI;
    PID_turn_to_point_chain(24, 48, 10);
    PID_forward(10, 0.3, 0.2);
    check("turn to point chain: forward keeps facing the point", fabs(getInertial() - aim_to) < 1.5, getInertial(), aim_to);

    reset();
    PID_swing_chain(90, LEFT_SIDE, 10);
    double swing_handover = getInertial(), swing_rate = sim::rate;
    PID_forward(12, 0.3, 0.2);
    check("swing chain hands over 10 degrees early, still turning", swing_handover > 78 && swing_handover < 86 && swing_rate > 30, swing_handover, swing_rate);
    check("  forward after it keeps the swing's target 90", fabs(getInertial() - 90) < 1.5, getInertial(), 90);

    // The other _async turns: return right away, then do the same as the waiting ones
    reset();
    setHeading(350);
    double t_call = sim::t_ms;
    PID_turn_shortest_async(10, 0.5, 0.2);
    bool returned_moving = isMoving() && sim::t_ms == t_call;
    waitUntilDone();
    check("turn shortest async: returns right away, 350 -> 370", returned_moving && fabs(getInertial() - 370) < 1.5, returned_moving, getInertial());
    reset();
    t_call = sim::t_ms;
    PID_turn_to_point_async(24, 24, 0.5, 0.2);
    returned_moving = isMoving() && sim::t_ms == t_call;
    waitUntilDone();
    check("turn to point async: returns right away, faces 45", returned_moving && fabs(getInertial() - 45) < 1.5, returned_moving, getInertial());

    // The gentle start is skipped only while the robot is still driving, not after a turn in place
    reset();
    PID_turn_chain(90, 10);
    PID_forward_async(24, 0.3, 0.2);
    vexDelay(50);
    double power_after_turn = (sim::cmd[0] + sim::cmd[1]) / 2;
    waitUntilDone();
    check("forward after a chained turn still starts gently", power_after_turn < 50, power_after_turn, 50);
    reset();
    PID_drive_to_point_async(48, 0, 0.5, 0.2); // turns 90 degrees first, chained into the drive
    while (getInertial() < 85 && isMoving()) vexDelay(5); // that turn hands over 5 degrees early
    bool turn_handed_over = isMoving();
    vexDelay(50);
    double power_after_point_turn = (sim::cmd[0] + sim::cmd[1]) / 2;
    waitUntilDone();
    check("  so does a drive to point after its turn", turn_handed_over && power_after_point_turn < 50, turn_handed_over, power_after_point_turn);
    reset();
    PID_forward_chain(24, 3);
    PID_forward_async(-24, 0.3, 0.2); // still rolling forward, but this one goes backwards
    vexDelay(50);
    double power_reversing = (sim::cmd[0] + sim::cmd[1]) / 2;
    waitUntilDone();
    check("  so does a backward drive after a chained forward", power_reversing > -50, power_reversing, -50);
    reset();
    PID_forward_chain(24, 3);
    double handover_speed = getMotorRate();
    PID_forward_async(24, 0.3, 0.2);
    vexDelay(50);
    double speed_after_handover = getMotorRate();
    waitUntilDone();
    check("forward after a chained forward doesn't slow down", speed_after_handover >= handover_speed, speed_after_handover, handover_speed);

    // setHeading between chained movements: the next one keeps aiming the same real direction.
    // The robot is still turning at the hand-over, so expect the turn's target in the new numbers.
    reset();
    PID_turn_chain(90, 10);
    double jump = 0 - getInertial();
    setHeading(0);
    PID_forward(24, 0.3, 0.2);
    check("setHeading after a chained turn: forward keeps its direction", fabs(getInertial() - (90 + jump)) < 1.5, getInertial(), 90 + jump);
    reset();
    PID_turn_chain(90, 10);
    jump = 0 - getInertial();
    setHeading(0);
    PID_turn_relative(90, 0.5, 0.2);
    check("  relative turn counts from the turn's target", fabs(getInertial() - (180 + jump)) < 1.5, getInertial(), 180 + jump);

    if (TRACKING_SIDEWAYS_PORT >= 0) {
        reset();
        PID_forward_async(48, 0.3, 0.2);
        waitUntilTraveled(20);
        sim::bump(5); // another robot pushes ours 5 inches to the right
        waitUntilDone(); settle();
        check("sideways tracking wheel sees the robot get pushed", odometryError() < 0.5 && fabs(sim::x - 5) < 0.5, odometryError(), sim::x);
        reset();
        PID_drive_to_point_async(0, 48, 0.5, 0.2);
        waitUntilTraveled(20);
        sim::bump(6);
        waitUntilDone(); settle();
        check("  drive to point still gets there after the push", hypot(sim::x, sim::y - 48) < 1.5, sim::x, sim::y);
    }

    // ---------- Correcting the position from a wall ----------
    reset();
    PID_forward(12, 0.3, 0.2); settle();
    double y_before = getY();
    setX(5);
    check("setX changes only x", getX() == 5 && getY() == y_before, getX(), getY());
    setY(-3);
    check("setY changes only y", getX() == 5 && getY() == -3, getX(), getY());
    double dx = getX() - sim::x, dy = getY() - sim::y;
    PID_turn(90, 0.5, 0.2); PID_forward(12, 0.3, 0.2); settle();
    check("  odometry goes on from there", hypot(getX() - sim::x - dx, getY() - sim::y - dy) < 0.5, getX() - sim::x, dx);

    // Puts the simulated robot at (x, y) facing heading, and tells odometry a position that is
    // off by (off_x, off_y), as if the wheels had slipped
    auto placeRobot = [](double x, double y, double heading, double off_x, double off_y) {
        settle(); sim::x = x; sim::y = y;
        setPose(x + off_x, y + off_y, heading);
    };
    if (DISTANCE_FRONT_PORT < 0) {
        placeRobot(40, 10, 90, 3, -2);
        check("no distance sensor: reset from wall -> false", !resetXFromWall(FRONT_SENSOR, 72)
              && getX() == 43 && getY() == 8, getX(), getY());
    } else {
        placeRobot(40, 10, 90, 3, -2);
        bool ok = resetXFromWall(FRONT_SENSOR, 72);
        check("front sensor at the x wall: corrects x", ok && fabs(getX() - 40) < 0.01, ok, getX());
        check("  y stays as it was", getY() == 8, getY(), 8);
        placeRobot(-10, 50, 0, 3, -2);
        ok = resetYFromWall(FRONT_SENSOR, 72);
        check("front sensor at the y wall: corrects y", ok && fabs(getY() - 50) < 0.01 && getX() == -7, getY(), getX());
        placeRobot(30, -20, 0, -4, 2);
        ok = resetXFromWall(RIGHT_SENSOR, 72);
        check("right sensor at the x wall", ok && fabs(getX() - 30) < 0.01, ok, getX());
        placeRobot(-50, 20, 0, 4, 2);
        ok = resetXFromWall(LEFT_SENSOR, -72);
        check("left sensor at the -x wall", ok && fabs(getX() + 50) < 0.01, ok, getX());
        placeRobot(15, -45, 0, 1, -5);
        ok = resetYFromWall(BACK_SENSOR, -72);
        check("back sensor at the -y wall", ok && fabs(getY() + 45) < 0.01, ok, getY());
        placeRobot(40, 10, 103, 3, -2); // 13 degrees from straight on
        ok = resetXFromWall(FRONT_SENSOR, 72);
        check("front sensor at an angle (13 deg)", ok && fabs(getX() - 40) < 0.01, ok, getX());
        placeRobot(-20, 40, 190, -2, 4); // the back sensor looks 10 degrees right of +y
        ok = resetYFromWall(BACK_SENSOR, 72);
        check("back sensor at an angle, heading 190", ok && fabs(getY() - 40) < 0.01, ok, getY());
        placeRobot(30, 30, -80, 2, -3); // the right sensor looks 10 degrees up from +y
        ok = resetYFromWall(RIGHT_SENSOR, 72);
        check("right sensor at the y wall, heading -80", ok && fabs(getY() - 30) < 0.01, ok, getY());

        placeRobot(40, 10, 120, 3, -2);
        check("too big an angle (30 deg) -> false, unchanged", !resetXFromWall(FRONT_SENSOR, 72)
              && getX() == 43 && getY() == 8, getX(), getY());
        placeRobot(40, 10, 90, 3, -2);
        sim::obstacle[DISTANCE_FRONT_PORT] = 15; // another robot in front of the sensor
        check("sees a robot instead of the wall -> false", !resetXFromWall(FRONT_SENSOR, 72) && getX() == 43, getX(), 43);
        sim::obstacle[DISTANCE_FRONT_PORT] = 0;
        placeRobot(-60, 10, 90, 3, -2);
        check("wall too far to see -> false", !resetXFromWall(FRONT_SENSOR, 72) && getX() == -57, getX(), -57);
        placeRobot(40, 10, 90, 3, -2);
        sim::unplugged[DISTANCE_FRONT_PORT] = true;
        check("sensor unplugged -> false", !resetXFromWall(FRONT_SENSOR, 72) && getX() == 43, getX(), 43);
        check("  checkDevices finds it", !checkDevices(), 0, 0);
        sim::unplugged[DISTANCE_FRONT_PORT] = false;
        placeRobot(40, 10, 90, 3, -2);
        check("a reset on the wrong wall (y) -> false", !resetYFromWall(FRONT_SENSOR, 72) && getY() == 8, getY(), 8);

        placeRobot(65.5, 10, 90, 3, -2); // the front sensor is 0.5 inch from the wall
        check("too close to the wall to measure -> false", !resetXFromWall(FRONT_SENSOR, 72) && getX() == 68.5, getX(), 68.5);
        placeRobot(64.5, 10, 90, 3, -2); // 1.5 inches is fine
        ok = resetXFromWall(FRONT_SENSOR, 72);
        check("  1.5 inches from the wall is fine", ok && fabs(getX() - 64.5) < 0.01, ok, getX());
        placeRobot(-60, 10, 90, 0, 0); // the back sensor sees the wall at x = -72
        check("named wall behind the sensor -> false", !resetXFromWall(BACK_SENSOR, -40, 100) && getX() == -60, getX(), -60);
        placeRobot(40, 10, 90, 8, 0);
        check("8 inches off -> false by default", !resetXFromWall(FRONT_SENSOR, 72) && getX() == 48, getX(), 48);
        ok = resetXFromWall(FRONT_SENSOR, 72, 12);
        check("  but fine with max_change 12", ok && fabs(getX() - 40) < 0.01, ok, getX());
        placeRobot(0, 10, 90, 0, 0);
        PID_forward_async(30, 0.3, 0.2);
        waitUntilTraveled(10);
        sim::x += 3; // slipped: a reset now would fix it, if the robot weren't moving
        ok = resetXFromWall(FRONT_SENSOR, 72);
        double odometry_off = getX() - sim::x; // still about -3 (odometry is up to 10 ms behind)
        waitUntilDone();
        check("robot still moving -> false", !ok && fabs(odometry_off + 3) < 1, ok, odometry_off);
        settle();
        ok = resetXFromWall(FRONT_SENSOR, 72);
        check("  after it stopped: fine", ok && fabs(getX() - sim::x) < 0.05, ok, getX() - sim::x);
        placeRobot(40, 10, 60, 0, 0);
        PID_turn_async(120, 0.5, 0.2);
        waitUntilTraveled(30); // facing the wall at x = 72, turning in place
        ok = resetXFromWall(FRONT_SENSOR, 72);
        waitUntilDone();
        check("  turning in place counts as moving", !ok, ok, 0);

        // The whole idea: the wheels slip, the wall fixes it, and the next movement gets there
        placeRobot(0, 0, 0, 0, 0);
        PID_drive_to_point(48, 24, 0.5, 0.2); PID_turn(90, 0.5, 0.2); settle();
        sim::x += 4; // the robot slid 4 inches without the wheels noticing
        bool fixed = resetXFromWall(FRONT_SENSOR, 72);
        PID_drive_to_point(48, 48, 0.5, 0.2); settle();
        check("slipped, reset from the wall, then drive to a point", fixed && hypot(sim::x - 48, sim::y - 48) < 1, sim::x, sim::y);
    }

    // ---------- Gains that can be changed while the program runs, and movement results ----------
    reset();
    PID_turn(90, 0.5, 0.2);
    MovementResult r = lastMovementResult();
    check("result of turn to 90: done, small error, no timeout", !r.timed_out && fabs(r.error) < 0.5 && r.time_ms < 3000 && r.time_ms > 100, r.error, r.time_ms);
    check("  error is target - where it ended", fabs(r.error - (90 - getInertial())) < 0.01, r.error, 90 - getInertial());
    reset();
    PID_forward(24, 0.3, 0.2);
    double normal_overshoot = lastMovementResult().overshoot;
    reset();
    PIDGains saved_forward = forwardGains;
    forwardGains.kd = 0; // no braking: it should go past the target
    PID_forward(24, 0.3, 0.2);
    r = lastMovementResult();
    forwardGains = saved_forward;
    check("forward without kD overshoots more", r.overshoot > normal_overshoot + 0.5, r.overshoot, normal_overshoot);
    check("  overshoot = how far past 24 it went", r.overshoot > 0.5 && r.overshoot < 24, r.overshoot, 0);
    reset();
    forwardGains = {0, 0, 0}; // no power at all
    PID_forward(24, 0.3, 0.2, 500);
    r = lastMovementResult();
    forwardGains = saved_forward;
    check("forward with all gains 0: doesn't move, times out", r.timed_out && fabs(sim::y) < 0.5 && fabs(r.error - 24) < 0.5, r.timed_out, sim::y);
    reset();
    PID_forward_async(24, 0.3, 0.2);
    // Change the gains right away, before the background task has even picked up the movement:
    // the movement must still use the gains from when it was started
    forwardGains = {0, 0, 0};
    waitUntilDone();
    forwardGains = saved_forward;
    check("gains changed after start: movement keeps its own", fabs(sim::y - 24) < 1, sim::y, 24);

    reset();
    PID_drive_to_point(0, -24, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, true);
    r = lastMovementResult();
    check("backwards drive to point: overshoot is small, not 24", r.overshoot < 1 && fabs(sim::y + 24) < 1, r.overshoot, sim::y);

    // A drive to point that stops during its turn-first still reports inches left to drive
    auto pointAhead = [](double x, double y) {
        double h = getInertial() * M_PI / 180;
        return (x - getX()) * sin(h) + (y - getY()) * cos(h);
    };
    reset();
    PID_drive_to_point_async(48, 0, 0.5, 0.2); // turns 90 degrees first
    vexDelay(100);
    cancelMovement();
    r = lastMovementResult();
    // (the robot still turns a little while it brakes, so allow some inches; in degrees it would be ~65)
    check("drive to point cancelled while turning: error in inches", fabs(r.error - pointAhead(48, 0)) < 4 && r.overshoot == 0 && !r.timed_out, r.error, pointAhead(48, 0));
    reset();
    forwardGains.kd = 0;
    PID_forward(24, 0.01, 0.01, 1500); // no braking, impossible tolerances: overshoots and times out
    forwardGains = saved_forward;
    MovementResult old_result = lastMovementResult();
    reset();
    PID_drive_to_point_async(0, 48, 0.5, 0.2); // straight ahead: no turn first
    cancelMovement();                          // before the background task even starts it
    r = lastMovementResult();
    check("  cancelled before it started: not the old result", old_result.timed_out && old_result.overshoot > 0.5
          && fabs(r.error - 48) < 0.5 && !r.timed_out && r.overshoot == 0, r.error, old_result.overshoot);
    reset();
    PID_drive_to_point(48, 0, 0.5, 0.2, 200); // 200 ms is too short for its 90 degree turn
    r = lastMovementResult();
    check("drive to point timing out while turning", r.timed_out && r.time_ms < 230 && fabs(r.error - pointAhead(48, 0)) < 1.5, r.time_ms, r.error);

    // The tuner's "every other try goes back" must bring the robot back to its spot
    reset();
    PID_arc(90, 24, 0.5, 0.2); PID_arc(0, -24, 0.5, 0.2); settle();
    check("arc 90 on R24, then back on R-24 -> start", hypot(sim::x, sim::y) < 1.5 && fabs(getInertial()) < 1.5, hypot(sim::x, sim::y), getInertial());
    reset();
    PID_swing(90, LEFT_SIDE, 0.5, 0.2); PID_swing(0, LEFT_SIDE, 0.5, 0.2); settle();
    check("swing to 90 and back with the left side -> start", hypot(sim::x, sim::y) < 1.5 && fabs(getInertial()) < 1.5, hypot(sim::x, sim::y), getInertial());

    // ---------- Tuning from the controller ----------
    reset();
    task operator_task([]() -> int {
        auto press = [](button &b, double ms) { b.down = true; vexDelay(ms); b.down = false; vexDelay(300); };
        auto waitForMove = [] { vexDelay(300); while (isMoving()) vexDelay(10); vexDelay(300); };
        vexDelay(300);
        press(Controller.ButtonRight, 300); // TURN kP 3.2 -> 3.5
        press(Controller.ButtonA, 300);     // try: turn to 90
        waitForMove();
        press(Controller.ButtonX, 300);     // FORWARD
        press(Controller.ButtonA, 300);     // try: drive 24
        waitForMove();
        press(Controller.ButtonLeft, 1300); // FORWARD kP, held: smaller a few times
        press(Controller.ButtonDown, 300);  // kI
        press(Controller.ButtonDown, 300);  // kD
        press(Controller.ButtonRight, 300); // FORWARD kD 0.78 -> 0.86
        press(Controller.ButtonB, 300);     // done
        return 0;
    });
    PIDGains saved_turn = turnGains;
    double tune_start = sim::t_ms;
    Brain.SDcard.inserted = true; // B saves the gains on it
    tuneWithController();
    printf("   tuner: turn kP %.3g, forward kP %.3g kD %.3g, robot (%.2f, %.2f) facing %.1f\n",
           turnGains.kp, forwardGains.kp, forwardGains.kd, sim::x, sim::y, getInertial());
    check("tuner: Right makes turn kP 10% bigger (3.2 -> 3.5)", fabs(turnGains.kp - 3.5) < 1e-9, turnGains.kp, 3.5);
    check("  A tried a turn to 90, X + A a 24 inch drive", fabs(getInertial() - 90) < 1.5 && fabs(sim::x - 24) < 1.5, getInertial(), sim::x);
    check("  holding Left lowers forward kP a few times", forwardGains.kp < 11 && forwardGains.kp > 5, forwardGains.kp, 12);
    check("  Down picks kD, Right makes it bigger", fabs(forwardGains.kd - 0.86) < 1e-9 && forwardGains.ki == saved_forward.ki, forwardGains.kd, forwardGains.ki);
    check("  B ends it", sim::t_ms - tune_start < 15000, sim::t_ms - tune_start, 0);
    std::string tuned = Brain.SDcard.files["pid_gains.txt"];
    check("  B saves the gains on the SD card", tuned.find("TURN_KP = 3.5 config 3.2\n") != std::string::npos
          && tuned.find("FORWARD_KD = 0.86 config 0.78\n") != std::string::npos, tuned.size(), 0);
    turnGains = saved_turn;
    forwardGains = saved_forward;

    // ---------- SD card: gains ----------
    auto sameGains = [](const PIDGains &a, const PIDGains &b) { return a.kp == b.kp && a.ki == b.ki && a.kd == b.kd; };
    const PIDGains config_turn = turnGains, config_forward = forwardGains, config_swing = swingGains, config_arc = arcGains;
    auto configGains = [&] { turnGains = config_turn; forwardGains = config_forward; swingGains = config_swing; arcGains = config_arc; };
    Brain.SDcard.files.erase("pid_gains.txt");
    Brain.SDcard.inserted = false;
    turnGains.kp = 4.4;
    check("no SD card: save -> false", !saveGainsToSDCard() && Brain.SDcard.files.empty(), Brain.SDcard.files.size(), 0);
    check("no SD card: load -> false, gains unchanged", !loadGainsFromSDCard() && turnGains.kp == 4.4, turnGains.kp, 4.4);
    Brain.SDcard.inserted = true;
    check("no pid_gains.txt yet: load -> false, gains unchanged", !loadGainsFromSDCard() && turnGains.kp == 4.4, turnGains.kp, 4.4);
    forwardGains.kd = 0.0123;
    arcGains.ki = 0;
    swingGains.kp = 1.0 / 3;
    bool saved = saveGainsToSDCard();
    configGains();
    bool loaded = loadGainsFromSDCard();
    printf("%s", Brain.SDcard.files["pid_gains.txt"].c_str());
    check("save, then load: the saved gains are back", saved && loaded && turnGains.kp == 4.4 && forwardGains.kd == 0.0123
          && arcGains.ki == 0 && fabs(swingGains.kp - 1.0 / 3) < 1e-9, turnGains.kp, forwardGains.kd);
    check("  the others stay as they were", sameGains(turnGains, {4.4, config_turn.ki, config_turn.kd})
          && forwardGains.kp == config_forward.kp && sameGains(arcGains, {config_arc.kp, 0, config_arc.kd}),
          forwardGains.kp, config_forward.kp);
    // A file changed by hand, and a config changed since the file was saved
    configGains();
    char edited[300];
    snprintf(edited, sizeof(edited),
             "# comment\r\n"
             "TURN_KP = 4.4 config %g\r\n"   // the config said something else back then: config wins
             "FORWARD_KP = 9 config %g\r\n"  // config unchanged: the file wins
             "TURN_KD=40\r\n"                // typed by hand, without the config part
             "FORWARD_KD = -1 config %g\r\n" // makes no sense: ignored
             "ARC_KI = nan\r\n"
             "LIFT_KP = 2\r\n",              // not ours: ignored
             TURN_KP + 1, FORWARD_KP, FORWARD_KD);
    Brain.SDcard.files["pid_gains.txt"] = edited;
    loaded = loadGainsFromSDCard();
    check("edited file: config changed since saving -> config", loaded && turnGains.kp == TURN_KP, turnGains.kp, TURN_KP);
    check("  config unchanged -> the file's value", forwardGains.kp == 9, forwardGains.kp, 9);
    check("  typed by hand without the config part", turnGains.kd == 40, turnGains.kd, 40);
    check("  -1 and nan are ignored", forwardGains.kd == config_forward.kd && arcGains.ki == config_arc.ki, forwardGains.kd, arcGains.ki);
    configGains();
    // Too big to read all of it: half a line could slip through, so none of it is used
    std::string big_file = std::string(2100, '#') + "\nTURN_KP = 4.4\n";
    Brain.SDcard.files["pid_gains.txt"] = big_file;
    check("gains file over 2 KB -> false, gains unchanged", !loadGainsFromSDCard() && turnGains.kp == config_turn.kp, turnGains.kp, config_turn.kp);

    // ---------- SD card: logging ----------
    // One table: move,name,time_ms,target,error,speed,output,p,i,d,x,y,heading
    struct LogLine { int move; std::string name; double time, error, x, y; int fields; };
    auto readLog = [](const std::string &text) {
        std::vector<LogLine> lines;
        size_t start = text.find('\n') + 1; // skip the header
        while (start < text.size()) {
            size_t end = text.find('\n', start);
            std::string line = text.substr(start, end - start);
            LogLine l = {0, "", 0, 0, 0, 0, 1};
            for (char c : line) l.fields += (c == ',');
            char name[64] = "";
            double target, speed, output, p, i, d, heading;
            sscanf(line.c_str(), "%d,%63[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &l.move, name, &l.time,
                   &target, &l.error, &speed, &output, &p, &i, &d, &l.x, &l.y, &heading);
            l.name = name;
            lines.push_back(l);
            start = end + 1;
        }
        return lines;
    };
    Brain.SDcard.inserted = true;
    for (int n = 1; n <= 9999; n++) Brain.SDcard.files["pidlog" + std::to_string(n) + ".csv"] = "full";
    check("logToSDCard: pidlog1 to pidlog9999 all taken -> false, nothing overwritten",
          !logToSDCard(true) && Brain.SDcard.files["pidlog9999.csv"] == "full", 0, 0);
    for (int n = 1; n <= 9999; n++) Brain.SDcard.files.erase("pidlog" + std::to_string(n) + ".csv");
    Brain.SDcard.files["pidlog1.csv"] = "from an earlier run";
    Brain.SDcard.inserted = false;
    check("logToSDCard without a card -> false", !logToSDCard(true), 0, 0);
    Brain.SDcard.inserted = true;
    check("logToSDCard: a new file, pidlog2.csv", logToSDCard(true) && Brain.SDcard.files["pidlog1.csv"] == "from an earlier run"
          && Brain.SDcard.files["pidlog2.csv"] == "move,name,time_ms,target,error,speed,output,p,i,d,x,y,heading\n",
          Brain.SDcard.files.size(), 3);
    reset();
    Brain.SDcard.fastest_write_speed = 0;
    int writes_before = Brain.SDcard.writes;
    PID_forward_chain(24, 3);
    vexDelay(40); // your code between two movements, the robot keeps driving meanwhile
    PID_turn_chain(90, 10);
    PID_forward(24, 0.3, 0.2);
    settle();
    std::vector<LogLine> log = readLog(Brain.SDcard.files["pidlog2.csv"]);
    int moves = log.empty() ? 0 : log.back().move;
    bool all_fields = true;
    for (const LogLine &l : log) all_fields = all_fields && l.fields == 13;
    printf("   log: %d lines, %d moves, %d writes, fastest wheel during a write %.1f in/s\n", (int)log.size(), moves,
           Brain.SDcard.writes - writes_before, Brain.SDcard.fastest_write_speed);
    check("log of a chained route: 3 moves, every line complete", moves == 3 && log.size() > 50 && all_fields, moves, log.size());
    check("  written only after the robot stopped", Brain.SDcard.writes > writes_before && Brain.SDcard.fastest_write_speed < 1,
          Brain.SDcard.writes - writes_before, Brain.SDcard.fastest_write_speed);
    check("  last line: forward, error ~0, x/y where the robot is", log.size() > 0 && log.back().name == "PID_forward"
          && fabs(log.back().error) < 0.5 && fabs(log.back().x - getX()) < 0.5 && fabs(log.back().y - getY()) < 0.5,
          log.empty() ? 0 : log.back().x, getX());
    bool every_20ms = log.size() > 0 && log[0].time == 0;
    for (size_t k = 1; k < log.size(); k++) {
        if (log[k].move == log[k - 1].move) {
            every_20ms = every_20ms && log[k].time - log[k - 1].time >= 19.9 && log[k].time - log[k - 1].time < 30;
        } else {
            every_20ms = every_20ms && log[k].time == 0; // each movement starts at 0
        }
    }
    check("  one line every 20 ms, from 0", every_20ms, 0, 0);
    // Longer than the memory buffer holds: it has to write in the middle, without losing lines
    reset();
    t0 = sim::t_ms;
    PID_forward(150, 0.3, 0.2, 12000, 20);
    double long_time = sim::t_ms - t0;
    settle();
    log = readLog(Brain.SDcard.files["pidlog2.csv"]);
    int long_lines = 0;
    all_fields = true;
    for (const LogLine &l : log) {
        if (l.move == 4) long_lines++;
        all_fields = all_fields && l.fields == 13;
    }
    check("11 s movement: every line is in the file", fabs(long_lines - long_time / 20) < 3 && all_fields
          && Brain.SDcard.files["pidlog2.csv"].size() > 40000, long_lines, long_time / 20);
    // Absurd numbers make a line too long for the buffer's line: left out, the file stays readable
    reset();
    PID_forward(1e250, 0.3, 0.2, 100);
    settle();
    log = readLog(Brain.SDcard.files["pidlog2.csv"]);
    all_fields = true;
    for (const LogLine &l : log) all_fields = all_fields && l.fields == 13 && l.move <= 4;
    check("huge target: its lines are left out, the file is fine", all_fields, log.size(), 0);
    // The card is taken out during a long movement: logging stops, it doesn't keep trying
    reset();
    task card_task([]() -> int { vexDelay(1000); Brain.SDcard.inserted = false; return 0; });
    int tries_before = Brain.SDcard.tries;
    PID_forward(150, 0.3, 0.2, 12000, 20);
    settle();
    check("card taken out: one failed write, then logging stops", Brain.SDcard.tries - tries_before == 1,
          Brain.SDcard.tries - tries_before, 1);
    Brain.SDcard.inserted = true;
    check("  logToSDCard(true) carries on in the same file", logToSDCard(true), 0, 0);
    logToSDCard(false);
    size_t log_size = Brain.SDcard.files["pidlog2.csv"].size();
    PID_turn(0, 0.5, 0.2);
    settle();
    check("logToSDCard(false): nothing more", Brain.SDcard.files["pidlog2.csv"].size() == log_size, Brain.SDcard.files["pidlog2.csv"].size(), log_size);

    // ---------- Mechanisms: an arm that holds, an intake that unjams ----------
    {
        sim::mech_load[ARM] = 15; // a heavy arm: gravity pulls with 15% of the motor's power
        arm.resetPosition(0);
        check("arm: resetPosition", fabs(arm.position()) < 0.01, arm.position(), 0);
        double t_start = sim::t_ms, highest_seen = 0;
        arm.moveTo(300);
        check("arm: not done right after moveTo", !arm.isDone(), arm.position(), 300);
        while (!arm.isDone()) { highest_seen = fmax(highest_seen, arm.position()); vexDelay(5); }
        check("arm: moveTo 300 against gravity arrives", fabs(arm.position() - 300) < ARM_TOLERANCE
              && sim::t_ms - t_start < 1500, arm.position(), sim::t_ms - t_start);
        check("  overshoot under 10 degrees", highest_seen - 300 < 10, highest_seen - 300, 10);
        bool arrived = arm.waitUntilDone();
        check("  waitUntilDone says it arrived", arrived, arrived, 1);
        vexDelay(2000);
        check("  holds there with the motor's hold mode", fabs(arm.position() - 300) < 2
              && sim::mech_mode[ARM] == 3, arm.position(), sim::mech_mode[ARM]);
        arm.moveTo(300);
        check("  moveTo the same position again: still done", arm.isDone(), arm.isDone(), 1);

        double lowest_seen = 1e9;
        arm.moveTo(50);
        while (!arm.isDone()) { lowest_seen = fmin(lowest_seen, arm.position()); vexDelay(5); }
        check("arm: moveTo 50, gravity helping", fabs(arm.position() - 50) < ARM_TOLERANCE, arm.position(), 50);
        check("  overshoot under 10 degrees", 50 - lowest_seen < 10, 50 - lowest_seen, 10);

        arm.moveTo(150, 30);
        double fastest = 0;
        while (!arm.isDone()) { fastest = fmax(fastest, sim::mech_cmd[ARM]); vexDelay(5); }
        check("arm: max_speed 30 limits the power", fastest <= 30.01 && fabs(arm.position() - 150) < ARM_TOLERANCE, fastest, arm.position());
        arm.moveTo(80, -30); // a negative speed is still a speed limit, not a direction
        fastest = 0;
        while (!arm.isDone()) { fastest = fmax(fastest, fabs(sim::mech_cmd[ARM])); vexDelay(5); }
        check("  max_speed -30 works like 30", fastest <= 30.01 && fabs(arm.position() - 80) < ARM_TOLERANCE, fastest, arm.position());

        // Driver control: a button that calls moveTo every loop, and manual(0) from an idle stick
        t_start = sim::t_ms;
        for (int i = 0; i < 150 && (i < 3 || !arm.isDone()); i++) { arm.moveTo(100); arm.manual(0); vexDelay(20); }
        check("arm: moveTo + manual(0) every loop still arrives", fabs(arm.position() - 100) < ARM_TOLERANCE
              && arm.isDone(), arm.position(), sim::t_ms - t_start);

        driveArm(60, 400);
        double moved_to = arm.position();
        check("arm: manual(60) raises it", moved_to > 150, moved_to, 100);
        driveArm(0, 1000);
        check("  manual(0): holds where it was let go", fabs(arm.position() - moved_to) < 10
              && sim::mech_mode[ARM] == 3, arm.position(), moved_to);
        driveArm(60, 200);
        moved_to = arm.position();
        driveArm(3, 1000); // a stick that doesn't sit exactly at 0
        check("  a stick inside the deadband holds too", fabs(arm.position() - moved_to) < 10
              && sim::mech_mode[ARM] == 3, arm.position(), moved_to);

        arm.moveTo(350);
        vexDelay(50);
        driveArm(-50, 100);
        check("  the stick takes over a running moveTo: isDone", arm.isDone(), arm.isDone(), 1);
        driveArm(0, 100);

        arm.setLimits(0, 400);
        arm.moveTo(1000);
        bool limited_ok = arm.waitUntilDone();
        check("arm: setLimits(0, 400), moveTo(1000) stops at 400", fabs(arm.position() - 400) < ARM_TOLERANCE
              && limited_ok, arm.position(), 400);
        arm.moveTo(200); arm.waitUntilDone();
        driveArm(100, 1500);
        check("  manual(100) stops at the highest", arm.position() < 410 && arm.position() > 390
              && sim::mech_mode[ARM] == 3, arm.position(), 400);
        int not_holding = 0; // the held arm sags a little below 400: it must not start driving up again
        for (int i = 0; i < 50; i++) { arm.manual(100); vexDelay(10); if (sim::mech_mode[ARM] != 3) not_holding++; }
        check("  keeps holding while the stick pushes up", not_holding == 0, not_holding, 0);
        driveArm(-100, 200);
        check("  manual(-100) still goes down from there", arm.position() < 370, arm.position(), 400);
        driveArm(-100, 1500);
        check("  ... and stops at the lowest", arm.position() > -10 && arm.position() < 10, arm.position(), 0);
        driveArm(0, 100);
        arm.setLimits(-1e9, 1e9);

        sim::mech_jammed[ARM] = true; // stuck on something
        arm.moveTo(300);
        t_start = sim::t_ms;
        arrived = arm.waitUntilDone();
        check("arm: stuck -> waitUntilDone false after ARM_TIMEOUT_MS", !arrived
              && fabs(sim::t_ms - t_start - ARM_TIMEOUT_MS) < 50, sim::t_ms - t_start, ARM_TIMEOUT_MS);
        check("  then holds where it is", sim::mech_mode[ARM] == 3, sim::mech_mode[ARM], 3);
        sim::mech_jammed[ARM] = false;

        double before_release = arm.position();
        arm.release();
        vexDelay(500);
        check("arm: release lets it fall", arm.position() < before_release - 20
              && sim::mech_mode[ARM] == 1, arm.position(), before_release);
        sim::mech_load[ARM] = 0;

        // Two motors
        sim::mech_load[PORT16] = sim::mech_load[PORT17] = 10;
        lift.resetPosition(0);
        lift.moveTo(-200);
        arrived = lift.waitUntilDone();
        check("lift (2 motors): moveTo -200", arrived && fabs(lift.position() + 200) < ARM_TOLERANCE
              && fabs(sim::mech_pos[PORT17] + 200) < ARM_TOLERANCE, lift.position(), sim::mech_pos[PORT17]);
        lift.release();
        sim::mech_load[PORT16] = sim::mech_load[PORT17] = 0;

        // Intake
        intake.spin(100);
        vexDelay(500);
        check("intake: spin(100) runs, starting up is not a jam", sim::mech_v[INTAKE] > 0.9 * sim::MECH_MAXV
              && !intake.isJammed() && intake.jamCount() == 0, sim::mech_v[INTAKE], intake.jamCount());
        sim::mech_jammed[INTAKE] = true;
        vexDelay(80);
        sim::mech_jammed[INTAKE] = false;
        vexDelay(300);
        check("  stuck for 80 ms: not a jam", intake.jamCount() == 0 && sim::mech_cmd[INTAKE] > 0, intake.jamCount(), 0);
        sim::mech_jammed[INTAKE] = true;
        vexDelay(INTAKE_JAM_MS - 30);
        check("  jammed: not noticed before INTAKE_JAM_MS", !intake.isJammed(), intake.isJammed(), 0);
        vexDelay(60);
        check("  noticed after it", intake.isJammed() && intake.jamCount() == 1, intake.isJammed(), intake.jamCount());
        check("  runs the other way to free it", sim::mech_cmd[INTAKE] < -99, sim::mech_cmd[INTAKE], -100);
        sim::mech_jammed[INTAKE] = false; // the piece comes loose
        for (int i = 0; i < 20; i++) { intake.spin(100); vexDelay(20); } // driver loop: same power again
        check("  then forward again", !intake.isJammed() && sim::mech_cmd[INTAKE] > 99
              && sim::mech_v[INTAKE] > 0.9 * sim::MECH_MAXV, sim::mech_cmd[INTAKE], intake.jamCount());
        sim::mech_jammed[INTAKE] = true;
        vexDelay(INTAKE_JAM_MS + INTAKE_UNJAM_MS + INTAKE_JAM_MS + 50);
        check("  stays stuck: tries again and again", intake.jamCount() == 3, intake.jamCount(), 3);
        sim::mech_jammed[INTAKE] = false;
        intake.spin(-100);
        vexDelay(300);
        check("  spin(-100) goes the other way", sim::mech_v[INTAKE] < -0.9 * sim::MECH_MAXV, sim::mech_v[INTAKE], 0);

        sim::mech_load[INTAKE] = 85; // heavy, but still turning
        intake.spin(100);
        vexDelay(1000);
        // (the current really is over the limit, only the speed says it isn't jammed)
        check("intake: heavy but still turning is not a jam", intake.jamCount() == 3
              && sim::mechCurrent(INTAKE) > INTAKE_JAM_CURRENT && sim::mech_v[INTAKE] / sim::MECH_MAXV * 100 > INTAKE_JAM_SPEED,
              sim::mechCurrent(INTAKE), sim::mech_v[INTAKE] / sim::MECH_MAXV * 100);
        sim::mech_load[INTAKE] = 0;
        intake.spin(5);
        sim::mech_jammed[INTAKE] = true;
        vexDelay(1000);
        check("intake: slow on purpose (5%) is not a jam", intake.jamCount() == 3, intake.jamCount(), 3);
        sim::mech_jammed[INTAKE] = false;

        intake.spin(100, false);
        vexDelay(300);
        sim::mech_jammed[INTAKE] = true;
        vexDelay(INTAKE_JAM_MS + 50);
        check("intake unjam=false: a jam stops it", intake.isJammed() && sim::mech_mode[INTAKE] == 1
              && intake.jamCount() == 4, intake.isJammed(), sim::mech_mode[INTAKE]);
        sim::mech_jammed[INTAKE] = false;
        for (int i = 0; i < 20; i++) { intake.spin(100, false); vexDelay(20); }
        check("  spin with the same power: stays stopped", intake.isJammed() && sim::mech_mode[INTAKE] == 1,
              intake.isJammed(), sim::mech_mode[INTAKE]);
        intake.stop();
        vexDelay(20);
        check("  stop(): not jammed any more", !intake.isJammed(), intake.isJammed(), 0);
        intake.spin(100, false);
        vexDelay(300);
        check("  spin again: runs", sim::mech_v[INTAKE] > 0.9 * sim::MECH_MAXV, sim::mech_v[INTAKE], 0);
        intake.stop();

        // Two motors: INTAKE_JAM_CURRENT is for each motor, not all of them together
        roller.spin(50);
        vexDelay(300);
        sim::mech_jammed[PORT19] = sim::mech_jammed[PORT20] = true; // each draws 1.35 A at 50%
        vexDelay(500);
        check("roller (2 motors): 1.35 A each at 50% is under the limit", roller.jamCount() == 0, roller.jamCount(), 0);
        roller.spin(100);
        vexDelay(INTAKE_JAM_MS + 50);
        check("  2.5 A each at 100%: jammed", roller.jamCount() == 1, roller.jamCount(), 1);
        sim::mech_jammed[PORT19] = sim::mech_jammed[PORT20] = false;
        roller.stop();
        vexDelay(50);

        // Only 8 of each: one more must not hang waitUntilDone, and must not move its motor
        static motor spare(PORT21, ratio36_1, false);
        for (int i = 0; i < 6; i++) { new Arm(spare); new Intake(spare); } // with the 2 above: 8 each
        Arm *ninth_arm = new Arm(spare);
        Intake *ninth_intake = new Intake(spare);
        t_start = sim::t_ms;
        ninth_arm->moveTo(100);
        arrived = ninth_arm->waitUntilDone();
        check("9th Arm: waitUntilDone returns false right away", !arrived && sim::t_ms - t_start < 1, arrived, sim::t_ms - t_start);
        ninth_intake->spin(100);
        vexDelay(100);
        check("9th Intake: does nothing", sim::mech_cmd[PORT21] == 0 && sim::mech_mode[PORT21] == 0, sim::mech_cmd[PORT21], 0);
    }

    // ---------- Robot setup checks ----------
    settle();
    int rumbles = Controller.rumbles;
    check("checkDevices: all plugged in -> true", checkDevices() && Controller.rumbles == rumbles, Controller.rumbles, rumbles);
    sim::unplugged[PORT_LEFTMIDDLE] = true;
    check("checkDevices: motor unplugged -> false, rumble", !checkDevices() && Controller.rumbles == rumbles + 1, Controller.rumbles, rumbles);
    sim::unplugged[PORT_LEFTMIDDLE] = false;
    sim::unplugged[PORT_INERTIAL] = true;
    check("checkDevices: inertial unplugged -> false", !checkDevices(), 0, 0);
    sim::unplugged[PORT_INERTIAL] = false;
    sim::heat[PORT_RIGHTBACK] = 25;
    check("checkDevices: motor at 60 C -> false", !checkDevices(), 0, 0);
    sim::heat[PORT_RIGHTBACK] = 0;

    reset();
    check("testDrivetrain: correct setup -> true", testDrivetrain(), 0, 0);
    reset();
    sim::wrong_direction[PORT_RIGHTMIDDLE] = true;
    check("testDrivetrain: one motor set the wrong way -> false", !testDrivetrain(), 0, 0);
    sim::wrong_direction[PORT_RIGHTMIDDLE] = false;
    reset();
    for (int p = 0; p < 3; p++) sim::wrong_direction[p] = true;
    check("testDrivetrain: whole left side the wrong way -> false", !testDrivetrain(), 0, 0);
    for (int p = 0; p < 3; p++) sim::wrong_direction[p] = false;
    reset();
    sim::sides_swapped = true;
    check("testDrivetrain: left and right swapped -> false", !testDrivetrain(), 0, 0);
    sim::sides_swapped = false;
    reset();
    sim::gyro_rate_flipped = true;
    check("testDrivetrain: gyro rate with the wrong sign -> false", !testDrivetrain(), 0, 0);
    sim::gyro_rate_flipped = false;
    reset();
    sim::unplugged[PORT_LEFTBACK] = true;
    check("testDrivetrain: motor unplugged -> false", !testDrivetrain(), 0, 0);
    sim::unplugged[PORT_LEFTBACK] = false;

    // ---------- Measuring the robot ----------
    reset();
    TrackWidthResult track = measureTrackWidth();
    check("measureTrackWidth: 12 inch track", fabs(track.track_width - 12) < 0.1, track.track_width, 12);
    check("  tracking wheel offsets (0 if none)", fabs(track.forward_offset - TRACKING_FORWARD_OFFSET) < 0.1
          && fabs(track.sideways_offset - TRACKING_SIDEWAYS_OFFSET) < 0.1, track.forward_offset, track.sideways_offset);
    reset();
    sim::track = 13; // the wheels slide while turning, so the drivetrain turns like a wider one
    track = measureTrackWidth();
    sim::track = 12;
    check("measureTrackWidth: robot that turns like a 13 inch track", fabs(track.track_width - 13) < 0.1, track.track_width, 13);

    // These change the wheel sizes in the simulator, which makes odometry jump: keep them last
    settle();
    sim::wheel_diam = 3.30;    // the config says 3.25
    sim::track_wheel_diam = TRACKING_WHEEL_DIAMETER_INCH + 0.05;
    task pusher_task([]() -> int {
        vexDelay(300);
        Controller.ButtonA.down = true; vexDelay(200); Controller.ButtonA.down = false; vexDelay(200);
        for (int i = 0; i < 48; i++) { sim::push(1); vexDelay(20); } // push it 48 inches
        Controller.ButtonA.down = true; vexDelay(200); Controller.ButtonA.down = false;
        return 0;
    });
    WheelSizeResult wheels = measureWheelSize(48);
    check("measureWheelSize: real wheels are 3.30, not 3.25", fabs(wheels.wheel_diameter - 3.30) < 0.01, wheels.wheel_diameter, 3.30);
    check("  tracking wheel (0 if none)", TRACKING_FORWARD_PORT < 0 ? wheels.tracking_wheel_diameter == 0
          : fabs(wheels.tracking_wheel_diameter - sim::track_wheel_diam) < 0.01, wheels.tracking_wheel_diameter, sim::track_wheel_diam);
    // The wheels are still bigger than the config says: the track width must use the measured size
    reset();
    track = measureTrackWidth();
    check("measureTrackWidth after measureWheelSize: still 12", fabs(track.track_width - 12) < 0.1, track.track_width, 12);
    check("  tracking wheel offsets with the measured wheel size", fabs(track.forward_offset - TRACKING_FORWARD_OFFSET) < 0.02
          && fabs(track.sideways_offset - TRACKING_SIDEWAYS_OFFSET) < 0.02, track.forward_offset, track.sideways_offset);

    printf("%d failures\n", fails);
    fflush(stdout);
    _exit(fails);
}
