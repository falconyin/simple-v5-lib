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

int main() {
    // Tell the simulator where the tracking wheels from simpleV5LibConfig.h are (if any)
    sim::fwd_wheel_port = TRACKING_FORWARD_PORT;
    sim::fwd_wheel_offset = TRACKING_FORWARD_OFFSET;
    sim::side_wheel_port = TRACKING_SIDEWAYS_PORT;
    sim::side_wheel_offset = TRACKING_SIDEWAYS_OFFSET;
    sim::track_wheel_diam = TRACKING_WHEEL_DIAMETER_INCH;
    printf("tracking wheels: forward port %d, sideways port %d\n", TRACKING_FORWARD_PORT, TRACKING_SIDEWAYS_PORT);

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
    logToSDCard(false);
    size_t log_size = Brain.SDcard.files["pidlog2.csv"].size();
    PID_turn(0, 0.5, 0.2);
    settle();
    check("logToSDCard(false): nothing more", Brain.SDcard.files["pidlog2.csv"].size() == log_size, Brain.SDcard.files["pidlog2.csv"].size(), log_size);

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
