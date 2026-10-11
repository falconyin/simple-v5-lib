#ifndef SIMPLEV5LIB_H
#define SIMPLEV5LIB_H
#include "simpleV5LibConfig.h"
#include <atomic>

using namespace vex;

// ============================================================================
// Devices (created in simpleV5lib.cpp, ports are set in simpleV5LibConfig.h)
// ============================================================================

extern brain Brain;
extern controller Controller;
extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;

// All motors of one side together, so one command drives the whole side
extern motor_group leftDrive;
extern motor_group rightDrive;

extern inertial Inertial;

// ============================================================================
// PID controller
// You can use this for your own mechanisms too (a lift, an arm, ...).
// ============================================================================

struct PIDController {
    double kp;
    double ki;
    double kd;
    double integral_range; // only add up the error when closer to the target than this

    double error_sum = 0;
    double past_error = 0;

    // The three parts of the last compute() result, handy for graphing (see logToTerminal)
    double last_p = 0;
    double last_i = 0;
    double last_d = 0;

    PIDController(double kp, double ki, double kd, double integral_range);

    // Call before starting a new movement
    void reset(double starting_error);

    // error: target - current position
    // rate:  how fast the position is changing right now (positive = position getting bigger)
    // Returns the power to use.
    double compute(double error, double rate);
};

// The kP, kI and kD of one kind of movement
struct PIDGains {
    double kp;
    double ki;
    double kd;
};

// The gains the movements use. They start with the values from simpleV5LibConfig.h.
// tuneWithController() changes them while the program runs, and you can change them in your own
// code too (for example a gentler turn while carrying something heavy). A movement uses the gains
// from the moment it starts, so a change only counts for the movements after it.
extern PIDGains turnGains;    // PID_turn, PID_turn_relative, PID_turn_shortest, PID_turn_to_point
extern PIDGains swingGains;   // PID_swing
extern PIDGains forwardGains; // PID_forward, PID_drive_to_point
extern PIDGains arcGains;     // PID_arc

// ============================================================================
// Setup
// ============================================================================

// Call once before autonomous (for example in pre_auton). Takes about 2 seconds.
// Keep the robot still while it runs. Afterwards odometry starts at (0, 0) facing 0.
void calibrateInertial();

// Tell the robot which way it is facing right now, in degrees.
// Useful at the start of an autonomous if the robot doesn't start facing 0.
// Fine between chained movements too: the next one keeps aiming the same real direction.
void setHeading(double degrees);

// ============================================================================
// Odometry: where is the robot on the field?
// Positions are in inches: x to the right, y forward (the way heading 0 points).
// It runs in the background and needs no extra code; tracking wheels are optional
// (see simpleV5LibConfig.h).
// ============================================================================

// Tell the robot where it is right now, for example at the start of an autonomous:
// setPose(0, 0, 0) = "here is (0, 0) and I'm facing 0 degrees"
void setPose(double x, double y, double heading);

double getX(); // inches
double getY(); // inches
// (the heading is getInertial())

// Change only x or only y, and keep the rest. For example after driving into a wall:
//   PID_forward(30, 0.3, 0.2, 1500, 40); // bump gently into the wall at y = 70 (stops at the timeout)
//   setY(70 - 7);                        // the robot's center is 7 inches from its front
void setX(double x);
void setY(double y);

// Correct the position from a wall, with a distance sensor (set it up in simpleV5LibConfig.h).
// wall_x / wall_y: where the wall is on the field. The sensor must look at that wall, roughly
// straight on (within DISTANCE_RESET_MAX_ANGLE degrees). Works at any heading.
//   resetXFromWall(RIGHT_SENSOR, 70);  // the right sensor sees the wall at x = 70: correct x
//   resetYFromWall(FRONT_SENSOR, 70);  // the front sensor sees the wall at y = 70: correct y
// Returns false and changes nothing (the terminal says why) if:
// - the robot is still moving (faster than DISTANCE_RESET_MAX_SPEED): the sensor is a little
//   behind, so stop first. A chained or _async movement may still be rolling.
// - the sensor sees nothing, or is too close to the wall to measure (under 20 mm)
// - it looks at the wall at too big an angle, or the wall is behind it (wrong sensor or wall?)
// - it says the position is more than max_change inches off: that's most likely another robot,
//   not the wall. max_change is DISTANCE_RESET_MAX_CHANGE unless you pass a bigger one, for
//   example after a crash: resetXFromWall(RIGHT_SENSOR, 70, 12);
// Closer walls are more exact (the sensor is about 0.6 inch off at 1 foot, more further away).
enum distanceSensor { FRONT_SENSOR, BACK_SENSOR, LEFT_SENSOR, RIGHT_SENSOR };
bool resetXFromWall(distanceSensor sensor, double wall_x, double max_change = DISTANCE_RESET_MAX_CHANGE);
bool resetYFromWall(distanceSensor sensor, double wall_y, double max_change = DISTANCE_RESET_MAX_CHANGE);

// Starts the odometry background task. calibrateInertial and setPose already call it.
void startOdometry();

// How far each tracking wheel has rolled, in inches (0 if you don't have that wheel)
double getForwardTrackingWheel();
double getSidewaysTrackingWheel();

// ============================================================================
// Sensors
// ============================================================================

double getInertial();  // heading in degrees, clockwise is positive, keeps counting past 360
double getGyroRate();  // turning speed in degrees/10 ms, clockwise is positive
double getPosition();  // distance driven in inches
double getMotorRate(); // driving speed in inches/s

// ============================================================================
// Basic driving
// ============================================================================

// left_speed and right_speed are percentages from -100 to 100
void move(double left_speed, double right_speed);

// Stop all drive motors. Uses DRIVE_BRAKE_MODE unless you pass a different one.
void stopDriving(brakeType mode = DRIVE_BRAKE_MODE);

// ============================================================================
// Autonomous movements
// Every movement waits until it is finished, then stops the drivetrain.
// Each one also has an _async version, see "Doing other things while driving" below.
//   error_tolerance: how close to the target counts as "there"
//   speed_tolerance: how slow the robot must be moving to count as stopped
//   timeout_ms:      (optional) give up after this many milliseconds
//   max_speed:       (optional) highest power to use, in percent (0 to 100)
// ============================================================================

// Drive straight. target in inches, negative drives backwards. Tolerances in inches and inches/s.
void PID_forward(double target, double error_tolerance, double speed_tolerance,
                 double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);

// Turn in place to an exact heading in degrees (see getInertial).
// PID_turn(450) after PID_turn(0) turns 450 degrees, it does not take a shortcut.
// Tolerances in degrees and degrees/10 ms.
void PID_turn(double target, double error_tolerance, double speed_tolerance,
              double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place by some degrees from where the robot is facing now. Positive = clockwise.
void PID_turn_relative(double degrees, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place to face a compass heading (0 to 360), taking the shorter way around.
void PID_turn_shortest(double heading, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Turn in place to face the field point (x, y), the shorter way around.
// backwards = true points the back of the robot at it. Tolerances in degrees and degrees/10 ms.
void PID_turn_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                       double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Drive to the field point (x, y), using odometry. Turns towards it first if needed, and keeps
// aiming at it while driving. backwards = true drives there in reverse.
// Tolerances in inches and inches/s.
void PID_drive_to_point(double x, double y, double error_tolerance, double speed_tolerance,
                        double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Which side of the drivetrain moves during a swing turn
enum driveSide { LEFT_SIDE, RIGHT_SIDE };

// Swing turn to an exact heading: only moving_side drives, the other side holds still.
// Tolerances in degrees and degrees/10 ms.
void PID_swing(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
               double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);

// Drive along a circle until the robot faces target (degrees, like PID_turn).
// radius: inches from the middle of the circle to the middle of the robot. Positive drives forward
// along the arc, negative drives backwards. Bigger radius = gentler curve, 0 = turn in place.
// Tolerances in degrees and degrees/10 ms. Needs TRACK_WIDTH_INCH in simpleV5LibConfig.h.
void PID_arc(double target, double radius, double error_tolerance, double speed_tolerance,
             double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);

// ============================================================================
// Doing other things while driving
// The _async versions start the movement and return right away, so your code can
// run an intake, raise a lift, ... while the robot drives. Same parameters as above.
// A new movement always waits for the previous one to finish first.
//
//   PID_forward_async(36, 0.3, 0.2);
//   waitUntilTraveled(12);       // after 12 inches...
//   intake.spin(100);            // ...start the intake (an Intake, see Mechanisms below)
//   waitUntilDone();             // wait for the drive to finish
// ============================================================================

void PID_forward_async(double target, double error_tolerance, double speed_tolerance,
                       double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_async(double target, double error_tolerance, double speed_tolerance,
                    double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_turn_relative_async(double degrees, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_turn_shortest_async(double heading, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_swing_async(double target, driveSide moving_side, double error_tolerance, double speed_tolerance,
                     double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_arc_async(double target, double radius, double error_tolerance, double speed_tolerance,
                   double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100, bool backwards = false);
void PID_drive_to_point_async(double x, double y, double error_tolerance, double speed_tolerance,
                              double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// Wait until the current movement is finished
void waitUntilDone();

// Wait until the current movement has gone this far from where it started
// (inches for PID_forward and PID_drive_to_point, degrees for turns, swings and arcs), or has finished
void waitUntilTraveled(double amount);

// true while a movement is running. A chained movement counts as finished when it hands over,
// even though the robot is still moving then.
bool isMoving();

// How a movement went
struct MovementResult {
    double time_ms;   // how long it took
    double error;     // how far from the target it ended: inches, or degrees for turns (like the graph's error)
    double overshoot; // the furthest it went past the target (0 = it never went past)
    bool timed_out;   // true if it gave up because it took longer than its timeout
};

// How the last movement went. If a movement is still running, waits for it to finish first.
//   PID_turn(90, 0.5, 0.2);
//   if (lastMovementResult().timed_out) { ... }
MovementResult lastMovementResult();

// Stop the current movement right away, or a chained one that is still rolling
// (also called by tankDrive / arcadeDrive)
void cancelMovement();

// ============================================================================
// Chained movements: don't stop between movements
// A chained movement finishes when it gets within exit_range of its target (inches for
// PID_forward_chain and PID_drive_to_point_chain, degrees for the others) and does NOT stop: the next movement takes
// over while the robot is still moving. Much faster, a little less exact.
// End your chain with a normal movement so the robot stops at the right place.
// (If nothing follows within CHAIN_STOP_AFTER_MS, the drivetrain stops on its own.)
// A turn in place (or a swing) after a chained drive first slows the robot down gently, so the
// wheels don't skid: the robot rolls on a few inches while it does. A drive to a point that has to
// curve sharply slows down for the corner the same way.
//
//   PID_forward_chain(24, 3);          // drive 24 inches, move on 3 inches before the end
//   PID_turn_chain(90, 10);            // turn to 90, move on 10 degrees before
//   PID_forward(24, 0.3, 0.2);         // normal movement: stops at the end
//
// With odometry, chain field points into a smooth path:
//   PID_drive_to_point_chain(0, 24, 4);   // move on 4 inches before (0, 24)
//   PID_drive_to_point_chain(24, 48, 4);  // curves towards the next point without stopping
//   PID_drive_to_point(48, 48, 0.5, 0.2); // stops exactly at the last point
// ============================================================================

void PID_forward_chain(double target, double exit_range, double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
void PID_turn_chain(double target, double exit_range, double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_swing_chain(double target, driveSide moving_side, double exit_range,
                     double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100);
void PID_arc_chain(double target, double radius, double exit_range,
                   double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100);
// exit_range in inches. Moves on when the point is less than exit_range ahead and the robot roughly
// faces it (within POINT_TURN_FIRST_ANGLE): a point beside the robot is driven to, not skipped.
// After another chained movement it curves towards the point instead of turning first (unless the
// point is more than 90 degrees around), so the robot never stops in the middle of the path.
void PID_drive_to_point_chain(double x, double y, double exit_range,
                              double timeout_ms = FORWARD_TIMEOUT_MS, double max_speed = 100, bool backwards = false);
// exit_range in degrees
void PID_turn_to_point_chain(double x, double y, double exit_range,
                             double timeout_ms = TURN_TIMEOUT_MS, double max_speed = 100, bool backwards = false);

// ============================================================================
// Live data, for tuning
// Shows what the PID is doing during every movement. Both are off by default.
// ============================================================================

// Print one line every 20 ms to the terminal (connect with a USB cable):
//   time_ms,error,speed,output,p,i,d
// Copy it into a spreadsheet and make a line chart to see the movement.
void logToTerminal(bool on);

// Save the same data to a file on the Brain's SD card, to look at after a match or a skills run,
// without a USB cable. Each program run makes a new file, with the first number that isn't taken
// yet: pidlog1.csv, pidlog2.csv, ... (so when you delete old logs, delete all of them, then the
// highest number is always the newest). It is one table, one line every 20 ms:
//   move,name,time_ms,target,error,speed,output,p,i,d,x,y,heading
// move counts the movements (1, 2, 3, ...), x, y and heading are where the robot was (odometry).
// Open it in a spreadsheet, filter on one move and make a line chart.
// The SD card is slow, so the data waits in memory and is written when the robot has stopped
// (between movements): take the card out only after the robot is done.
// Returns false (and logs nothing) if there is no SD card in the Brain.
bool logToSDCard(bool on);

// Draw the error (red) and the power (green) on the Brain screen while the robot moves.
// The middle line is 0: a red line that crosses it means the robot went past the target.
void graphOnScreen(bool on);

// Used by the movements to report their data
void telemetryStart(const char* name, double target, double start_error, double timeout_ms);
void telemetryUpdate(double time_ms, double error, double speed, double output, const PIDController &pid);
// Called by the background task while no movement is running: writes the logToSDCard data
void telemetryWriteSDCard();

// Tune the PID gains from the controller, without downloading the program again:
//   X              pick what to tune: turn, forward, swing or arc
//   Up / Down      pick kP, kI or kD
//   Right / Left   make it bigger / smaller (hold to keep changing)
//   A              try it: the robot turns 90 degrees (or drives 24 inches, ...), the next try goes back
//   B              done: saves the gains on the SD card (if one is in, see loadGainsFromSDCard),
//                  shows them on the Brain screen and the terminal, to copy into
//                  simpleV5LibConfig.h, and returns
// The controller screen shows the value and how the last try went: time, overshoot and error.
// THE ROBOT MOVES when you press A, give it room. See examples/tuning.
void tuneWithController();

// ============================================================================
// Keeping the tuned gains on the SD card
// Gains changed while the program runs (by tuneWithController() or your code) are lost when it
// stops. Save them on the Brain's SD card, and load them at the start of the next run:
//   loadGainsFromSDCard(); // at the start of main or pre_auton
// The file is pid_gains.txt, plain text: you can read it (and change it) on a computer too.
// ============================================================================

// Saves turnGains, swingGains, forwardGains and arcGains. Returns false if there is no SD card
// or the file could not be written. tuneWithController() does this for you when you press B.
bool saveGainsToSDCard();

// Puts the gains from pid_gains.txt into turnGains, swingGains, forwardGains and arcGains, and
// shows them in the terminal. Without an SD card or file (or if the file is over 2 KB) it returns false, and the gains stay
// the ones from simpleV5LibConfig.h.
// If you change a gain in simpleV5LibConfig.h after it was saved, the config wins for that gain:
// the file remembers what the config said when it was saved, so it can tell.
bool loadGainsFromSDCard();

// ============================================================================
// Robot setup checks: find setup mistakes before they cost you a match
// They show what they find on the Brain screen, the controller and the terminal.
// See examples/robotSetup.
// ============================================================================

// Checks that every motor and sensor from simpleV5LibConfig.h is plugged in, and that no drive
// motor is overheating. Doesn't move the robot. Returns true if everything is fine.
// Good to call in pre_auton: the controller rumbles if something is wrong.
bool checkDevices();

// Drives each side of the drivetrain forward for a moment, to check the motor setup:
// motors set to the wrong direction (LF_DIRECTION, ...), left and right mixed up, and whether the
// inertial sensor's turning speed has the right sign. Returns true if everything is fine.
// THE ROBOT MOVES (it turns a little each way), give it room.
bool testDrivetrain();

// Spins the robot in place 3 times and works out, with the inertial sensor, the TRACK_WIDTH_INCH
// your drivetrain really turns with (wheels slide a little while turning, so it is often a bit
// more than the tape measure says), and the TRACKING_..._OFFSET of your tracking wheels.
// Shows the numbers to put in simpleV5LibConfig.h. THE ROBOT MOVES, give it room.
// It counts inches with the wheel size, so run measureWheelSize() first: it uses that result
// (in the same program run), even before you have put the new wheel size in the config.
struct TrackWidthResult {
    double track_width;     // inches, 0 if the measurement failed
    double forward_offset;  // TRACKING_FORWARD_OFFSET (0 without a forward tracking wheel)
    double sideways_offset; // TRACKING_SIDEWAYS_OFFSET (0 without a sideways tracking wheel)
};
TrackWidthResult measureTrackWidth();

// Works out the real size of the wheels: press A (or tap the Brain screen), push the robot
// straight forward by hand exactly push_inches (along a tape measure), press A again.
// Shows the WHEEL_DIAMETER_INCH (and TRACKING_WHEEL_DIAMETER_INCH) to put in simpleV5LibConfig.h.
// A wrong MOTOR_TO_WHEEL_GEAR_RATIO shows up here too, as a wheel size that is way off.
struct WheelSizeResult {
    double wheel_diameter;          // inches, 0 if the measurement failed
    double tracking_wheel_diameter; // inches, 0 without a forward tracking wheel or if it failed
};
WheelSizeResult measureWheelSize(double push_inches);

// ============================================================================
// Driver control: call one of these inside the while loop in usercontrol()
// ============================================================================

// Left stick up/down drives the left side, right stick up/down drives the right side
void tankDrive();

// Left stick up/down drives forward and back, right stick left/right turns
void arcadeDrive();

// ============================================================================
// Mechanisms: an arm that holds its position, an intake that frees itself when it jams
// Make them once, next to your motors, outside any function (they must exist the whole program):
//   motor armMotor(PORT9, ratio36_1, false);
//   Arm arm(armMotor);                 // or Arm arm(armMotors) with a motor_group
//   motor intakeMotor(PORT10, ratio6_1, false);
//   Intake intake(intakeMotor);
// A background task moves them, so your code goes on right away, while driving too.
// Up to 8 Arms and 8 Intakes (more do nothing, and say so in the terminal).
// See examples/mechanisms.
// ============================================================================

// A lift or an arm: anything that moves to a position and has to stay there.
// Positions are in degrees of the motor (what position() says), 0 = where it was when the program
// started, or the number you gave resetPosition. Find your positions by moving the arm and reading
// position() on the Brain screen.
class Arm {
public:
    // kp, ki, kd: the PID gains for moving (power in percent per degree), ARM_KP, ... unless you pass others
    Arm(motor &one_motor, double kp = ARM_KP, double ki = ARM_KI, double kd = ARM_KD);
    Arm(motor_group &motors, double kp = ARM_KP, double ki = ARM_KI, double kd = ARM_KD);

    // Move to this position and stay there: the motor's hold mode keeps it there once it arrives.
    // Returns right away. Calling it again with the same position does nothing, so it is fine to
    // call it every time through the driver control loop while a button is pressed.
    void moveTo(double degrees, double max_speed = 100);

    // Wait until the arm got to where moveTo sent it. Returns false if it didn't make it within
    // ARM_TIMEOUT_MS (then it holds wherever it got to).
    bool waitUntilDone();
    bool isDone(); // true when it is there (or gave up), false while still moving

    // For driver control, call it every time through the loop: power in percent, from a joystick or
    // buttons. 0 = hold where it is now. While a moveTo is running, 0 lets it finish.
    // Close to a limit (setLimits) the power is eased off, so the arm can't fly past the limit.
    //   arm.manual(Controller.Axis2.position(percentUnits::pct));
    void manual(double power);

    // The arm can't go lower than lowest or higher than highest, with moveTo or manual.
    // Set them after resetPosition, in the same degrees.
    void setLimits(double lowest, double highest);

    // "The arm is at this position now", for example at the start with the arm resting all the
    // way down: resetPosition(0). Call it while the arm is not moving.
    void resetPosition(double degrees = 0);

    double position(); // degrees

    // Stop holding, let the arm hang loose (coast)
    void release();

    // Called every 10 ms by the background task, you don't need to call it
    void update();

private:
    motor_group own_motors; // used when the Arm was made with one motor
    motor_group *motors;
    PIDController pid;
    // Set by your code, picked up by the background task
    std::atomic<double> lowest;
    std::atomic<double> highest;
    std::atomic<int> request; // goes up by one for every new command
    std::atomic<int> command;
    std::atomic<double> target;
    std::atomic<double> max_speed;
    std::atomic<double> manual_power;
    int last_command;         // the last command your code gave, to ignore repeats
    bool registered;          // false if there were already 8 Arms: then this one does nothing
    // Set by the background task: the last request it has finished, and if that moveTo got there.
    // Done means finished_request == request, so an old move finishing can't count for a new one.
    std::atomic<int> finished_request;
    std::atomic<bool> arrived;
    void init();
    void send(int new_command);
    // Only used by the background task
    int seen_request = 0;
    int state;
    double start_time = 0;
    double settle_start = -1;
    int held_at_limit = 0; // 1 = holding at the highest because the stick pushes up, -1 = at the lowest
    void begin(int new_command);
};

// An intake (or a conveyor, a roller, ...): spins until told to stop, and notices when it is stuck.
// A jam is when the motors draw more than INTAKE_JAM_CURRENT amps (each), but turn slower than
// INTAKE_JAM_SPEED percent, for INTAKE_JAM_MS. A motor that is told to spin but can't turn draws
// a lot of current.
class Intake {
public:
    Intake(motor &one_motor);
    Intake(motor_group &motors);

    // Spin at this power, in percent (negative = the other way). Returns right away.
    // unjam = true:  when it jams, it runs the other way for INTAKE_UNJAM_MS, then goes on.
    // unjam = false: when it jams, it stops (and isJammed() says so) until you call stop() or spin
    //                with a different power. Handy to know a game piece is all the way in.
    // Fine to call every time through the driver control loop: the same power again changes nothing.
    void spin(double power, bool unjam = true);
    void stop();

    // true while it is jammed: running the other way to free it, or stopped (unjam = false)
    bool isJammed();
    // How many times it has jammed since the program started
    int jamCount();

    // Called every 10 ms by the background task, you don't need to call it
    void update();

private:
    motor_group own_motors;
    motor_group *motors;
    // Set by your code, picked up by the background task
    std::atomic<int> request;
    std::atomic<double> power;
    std::atomic<bool> unjam;
    std::atomic<bool> jammed; // set by the background task
    std::atomic<int> jams;    // set by the background task
    double last_power;        // the last power your code gave, to ignore repeats
    bool last_unjam;
    bool registered; // false if there were already 8 Intakes: then this one does nothing
    void init();
    // Only used by the background task
    int seen_request = 0;
    int state;
    double running_power = 0;
    double jam_start = -1;   // when it started to look jammed, -1 = it doesn't
    double unjam_start = 0;
};

// ============================================================================
// Autonomous selector
// Pick which autonomous to run from the Brain screen (tap left / right half)
// or the controller (Left / Right arrow buttons).
// ============================================================================

// Add a routine to the list, with a short name (up to 10 routines)
void addAuton(const char* name, void (*routine)());

// Call at the end of pre_auton, after addAuton
void startAutonSelector();

// Call at the start of usercontrol, so the arrow buttons are free for driving
void stopAutonSelector();

// Call in autonomous(): runs the routine that is selected
void runSelectedAuton();

#endif
