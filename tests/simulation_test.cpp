// Tests for simple-v5-lib, run on a computer against the simulator in tests/sim/vex.h.
// Build and run them with tests/run_tests.sh. Every check prints PASS or FAIL;
// the program exits with the number of failures, so 0 means everything passed.

#include "simpleV5lib.h"
#include <cstdio>
#include <unistd.h>
static int auton_runs = 0;
static void testAuton() { auton_runs++; }
static int fails = 0;
static void check(const char* name, bool ok, double a, double b) {
    printf("%-46s %s  (%.2f, %.2f)\n", name, ok ? "PASS" : "FAIL", a, b);
    if (!ok) fails++;
}
static void settle() { stopDriving(); vexDelay(500); }
int main() {
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
    auto reset = []{ settle(); setHeading(0); sim::x = 0; sim::y = 0; };
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

    printf("%d failures\n", fails);
    fflush(stdout);
    _exit(fails);
}
