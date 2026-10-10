#pragma once
// A tiny drivetrain simulator that stands in for the VEX SDK, so the library can be tested on a
// computer (see tests/run_tests.sh). It only has the parts of the VEX API the library uses.
//
// - Each side of the drivetrain responds to its voltage like a real motor (speed follows the
//   command with a 0.1 s delay). Ports 1-3 are the left side, 4-6 the right side. A side gets the
//   average of its three motors, so a motor that spins the wrong way slows its side down.
// - Setup mistakes can be switched on: a motor unplugged or set to the wrong direction, the left and
//   right side swapped, the gyro rate with the wrong sign, a wheel size that differs from the config.
// - The inertial sensor reports the heading worked out from the two wheel speeds.
// - Rotation sensors act as tracking wheels: the test tells the simulator which ports they are on
//   and where they sit on the robot. bump() shoves the robot sideways, like another robot would.
// - Distance sensors measure to the field walls (a square, 144 inches wide, centered on (0, 0)),
//   or to an obstacle the test puts in front of them.
// - Tasks are real threads, but only one runs at a time and they only switch inside vexDelay /
//   wait, like on the V5 brain. Time is simulated, so the tests run much faster than real time.
#define _USE_MATH_DEFINES // M_PI: the VEX SDK has it, but MinGW only declares it with this
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <cstdio>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <functional>
namespace sim {
inline double t_ms = 0;
inline double cmd[2] = {0, 0};     // commanded % per side (0 = left, 1 = right)
inline int mode[2] = {0, 0};       // 0 = voltage, 1 = coast, 2 = brake/hold
inline double v[2] = {0, 0};       // wheel speed in/s
inline double dist[2] = {0, 0};    // wheel travel in
inline double heading = 0;         // deg, clockwise positive
inline double rate = 0;            // deg/s clockwise
inline double gain[2] = {1, 1};    // side strength, to simulate a robot that pulls to one side
inline double peak = 0;            // highest wheel speed seen
inline double peak_cmd = 0;        // highest commanded power
inline double x = 0, y = 0;         // field position, heading 0 = +y, clockwise positive
// Tracking wheels (rotation sensors): which port each is on (-1 = none), where it sits, how far it rolled
inline int fwd_wheel_port = -1, side_wheel_port = -1;
inline double fwd_wheel_offset = 0;  // inches to the right of the robot's center
inline double side_wheel_offset = 0; // inches in front of the robot's center
inline double track_wheel_diam = 2.75;
inline double fwd_wheel = 0, side_wheel = 0; // inches rolled
// Setup mistakes (by port index: PORT1 = 0)
inline bool unplugged[21] = {};       // device not plugged in
inline bool wrong_direction[21] = {}; // motor spins the other way than the library thinks
inline double heat[21] = {};          // motor temperature above the normal 35 C
inline bool sides_swapped = false;    // the left motors' ports are really on the right side, and back
inline bool gyro_rate_flipped = false; // the gyro rate has the other sign than the library assumes
inline double wheel_diam = 3.25;      // the real drive wheel size (the config says 3.25)
inline double track = 12;             // the real track width
inline double motor_cmd[21] = {};     // the last power each motor was given
const double MAXV = 70, TAU = 0.1;
// Which side a drive motor port is on (0 = left, 1 = right)
inline int sideOf(int port) { return (port >= 3) != sides_swapped ? 1 : 0; }
// A side gets the average power of its three motors
inline void updateSide(int side) {
    double sum = 0;
    for (int p = 0; p < 6; p++) if (sideOf(p) == side) sum += motor_cmd[p];
    cmd[side] = sum / 3;
}
// The robot is pushed straight forward by hand (motors coasting)
inline void push(double inches) {
    dist[0] += inches; dist[1] += inches; fwd_wheel += inches;
    x += inches * sin(heading * M_PI / 180);
    y += inches * cos(heading * M_PI / 180);
}
inline void step();
// Another robot shoves ours sideways (positive = to the robot's right) without turning it
inline void bump(double right) {
    x += right * cos(heading * M_PI / 180);
    y -= right * sin(heading * M_PI / 180);
    side_wheel += right;
}
// The field walls (inches), for the distance sensors
inline double wall_left = -72, wall_right = 72, wall_back = -72, wall_front = 72;
// Distance sensors: where the sensor on each port sits on the robot (like DISTANCE_..._AHEAD and
// _RIGHT in the config) and which way it looks (degrees clockwise from the robot's front)
struct DistanceMount { bool on = false; double ahead = 0, right = 0, looks = 0; };
inline DistanceMount distance_mount[21];
inline double obstacle[21] = {}; // > 0: something this many inches in front of that sensor (another robot)
// What the distance sensor on this port sees, in inches, or -1 if nothing within its range
inline double distanceSeen(int port) {
    const DistanceMount &m = distance_mount[port];
    if (!m.on) return -1;
    double h = heading * M_PI / 180, b = (heading + m.looks) * M_PI / 180;
    double sx = x + m.ahead * sin(h) + m.right * cos(h), sy = y + m.ahead * cos(h) - m.right * sin(h);
    double dx = sin(b), dy = cos(b), seen = 1e9;
    if (dx > 1e-9) seen = fmin(seen, (wall_right - sx) / dx);
    if (dx < -1e-9) seen = fmin(seen, (wall_left - sx) / dx);
    if (dy > 1e-9) seen = fmin(seen, (wall_front - sy) / dy);
    if (dy < -1e-9) seen = fmin(seen, (wall_back - sy) / dy);
    if (obstacle[port] > 0) seen = fmin(seen, obstacle[port]);
    return seen <= 78 ? seen : -1; // the V5 distance sensor sees up to 2 m
}
inline void step() {               // 1 ms
    for (int s = 0; s < 2; s++) {
        double target = mode[s] == 0 ? cmd[s] / 100 * MAXV * gain[s] : 0;
        double tau = mode[s] == 0 ? TAU : (mode[s] == 1 ? 0.5 : 0.03);
        v[s] += (target - v[s]) * 0.001 / tau;
        dist[s] += v[s] * 0.001;
        peak = fmax(peak, fabs(v[s]));
    }
    rate = (v[0] - v[1]) / track * 180 / M_PI;
    heading += rate * 0.001;
    double vc = (v[0] + v[1]) / 2;
    x += vc * sin(heading * M_PI / 180) * 0.001;
    y += vc * cos(heading * M_PI / 180) * 0.001;
    // A wheel to the right of the center rolls backwards when the robot turns clockwise,
    // a wheel in front of the center rolls to the right
    double turn_rad = rate * M_PI / 180 * 0.001;
    fwd_wheel += vc * 0.001 - fwd_wheel_offset * turn_rad;
    side_wheel += side_wheel_offset * turn_rad;
    t_ms += 1;
}
}
namespace vex {
enum { PORT1=0,PORT2,PORT3,PORT4,PORT5,PORT6,PORT7,PORT8,PORT9,PORT10,PORT11,PORT12,PORT13,PORT14 };
enum gearSetting { ratio36_1, ratio18_1, ratio6_1 };
enum class rotationUnits { deg, rev };
enum class temperatureUnits { celsius, fahrenheit };
enum class velocityUnits { dps, pct };
enum class percentUnits { pct };
enum class voltageUnits { mV, volt };
enum class directionType { fwd, rev };
enum class brakeType { coast, brake, hold };
enum class timeUnits { sec, msec };
const timeUnits seconds = timeUnits::sec;
const timeUnits msec = timeUnits::msec;
enum axisType { xaxis, yaxis, zaxis };
enum rateUnits { dps };
struct color { int c; static const color red, green, white; };
inline const color color::red{1}, color::green{2}, color::white{3};
struct screen { int lines = 0, clears = 0; void clearScreen(){ clears++; } void setCursor(int,int){} int print(const char*, ...){return 0;}
  int printAt(int,int,const char*, ...){return 0;} int printAt(int,int,bool,const char*, ...){return 0;} void setPenColor(const color&){} void drawLine(int,int,int x2,int y2){ lines++; if (x2<0||x2>480||y2<0||y2>240) printf("OFFSCREEN %d %d\n", x2, y2); }
  bool pressing(){return false;} int xPosition(){return 0;} void clearLine(int){} };
// The SD card keeps its files in memory. writes counts the writes, and fastest_write_speed is the
// fastest a wheel was moving during any write (the library should only write when the robot stopped).
struct sdcard { bool inserted = false; int writes = 0, tries = 0; double fastest_write_speed = 0; std::map<std::string, std::string> files;
  bool isInserted(){ return inserted; }
  bool exists(const char *name){ return inserted && files.count(name) > 0; }
  int32_t size(const char *name){ return exists(name) ? (int32_t)files[name].size() : 0; }
  int32_t write(const char *name, uint8_t *buffer, int32_t len, bool append){
    tries++; if (!inserted) return 0;
    writes++; fastest_write_speed = fmax(fastest_write_speed, fmax(fabs(sim::v[0]), fabs(sim::v[1])));
    std::string &file = files[name]; if (!append) file.clear();
    file.append((const char*)buffer, len); return len; }
  int32_t savefile(const char *name, uint8_t *buffer, int32_t len){ return write(name, buffer, len, false); }
  int32_t appendfile(const char *name, uint8_t *buffer, int32_t len){ return write(name, buffer, len, true); }
  int32_t loadfile(const char *name, uint8_t *buffer, int32_t len){
    if (!exists(name)) return 0;
    int32_t n = (int32_t)fmin(len, files[name].size()); memcpy(buffer, files[name].data(), n); return n; } };
struct brain { double timer(timeUnits u){ return u == timeUnits::msec ? sim::t_ms : sim::t_ms / 1000; } screen Screen; sdcard SDcard; };
struct axis { int value = 0; int position(percentUnits){ return value; } };
struct button { bool down = false; bool pressing(){ return down; } };
struct controller { axis Axis1, Axis2, Axis3, Axis4; button ButtonLeft, ButtonRight, ButtonUp, ButtonDown, ButtonA, ButtonB, ButtonX, ButtonY;
  screen Screen; int rumbles = 0; void rumble(const char*){ rumbles++; } };
const double RATIO = 2.0 / 3.0;
struct motor { int port;
  motor(int p, gearSetting, bool) : port(p) {}
  int side(){ return sim::sideOf(port); }
  double sign(){ return sim::unplugged[port] ? 0 : (sim::wrong_direction[port] ? -1 : 1); }
  bool installed(){ return !sim::unplugged[port]; }
  double temperature(temperatureUnits){ return 35 + sim::heat[port]; }
  double position(rotationUnits){ return sign() * sim::dist[side()] / (sim::wheel_diam * M_PI) / RATIO; }
  double velocity(velocityUnits){ return sign() * sim::v[side()] / (sim::wheel_diam * M_PI) / RATIO * 360; }
  void stop(brakeType b){ sim::mode[side()] = b == brakeType::coast ? 1 : 2; }
  void spin(directionType, double mv, voltageUnits){
    double pct = fmax(-100, fmin(100, mv / 120));
    sim::mode[side()] = 0; sim::motor_cmd[port] = sign() * pct; sim::updateSide(side());
    sim::peak_cmd = fmax(sim::peak_cmd, fabs(pct)); } };
struct motor_group { std::vector<motor*> m; template<class... M> motor_group(M&... all) : m{&all...} {}
  void stop(brakeType b){ for (motor *x : m) x->stop(b); } void spin(directionType d, double x, voltageUnits u){ for (motor *mm : m) mm->spin(d, x, u); } };
struct rotation { int port; bool reversed;
  rotation(int p, bool r = false) : port(p), reversed(r) {}
  bool installed(){ return !sim::unplugged[port]; }
  double position(rotationUnits){
    double inches = port == sim::fwd_wheel_port ? sim::fwd_wheel : port == sim::side_wheel_port ? sim::side_wheel : 0;
    return inches / (M_PI * sim::track_wheel_diam) * (reversed ? -1 : 1); } };
enum class distanceUnits { mm, in, cm };
struct distance { int port; distance(int p) : port(p) {} bool installed(){ return sim::distance_mount[port].on && !sim::unplugged[port]; }
  bool isObjectDetected(){ return sim::distanceSeen(port) >= 0; }
  double objectDistance(distanceUnits u){ double in = sim::distanceSeen(port); if (in < 0) in = 9999 / 25.4;
    return u == distanceUnits::in ? in : u == distanceUnits::cm ? in * 2.54 : in * 25.4; } };
struct inertial { int port; inertial(int p) : port(p) {} bool installed(){ return !sim::unplugged[port]; }
  double rotation(rotationUnits){ return sim::heading; }
  double gyroRate(axisType, rateUnits){ return sim::gyro_rate_flipped ? sim::rate : -sim::rate; } // counter-clockwise positive, like the real IMU (assumed)
  void calibrate(){} bool isCalibrating(){ return false; }
  void setRotation(double d, rotationUnits){ sim::heading = d; } void setHeading(double, rotationUnits){} };
}
namespace sim {
// Cooperative scheduler: only one thread runs at a time, switching only inside vexDelay,
// and simulated time jumps to the earliest wake-up.
struct Thr { int id; double wake; };
inline std::mutex mu;
inline std::condition_variable cv;
inline std::vector<Thr> thr{{0, 0}};
inline int running = 0, next_id = 1;
inline thread_local int my_id = 0;
inline void handOff(std::unique_lock<std::mutex> &lk, bool wait_for_me) {
    size_t best = 0;
    for (size_t i = 1; i < thr.size(); i++)
        if (thr[i].wake < thr[best].wake || (thr[i].wake == thr[best].wake && thr[i].id < thr[best].id)) best = i;
    while (t_ms < thr[best].wake) step();
    running = thr[best].id;
    cv.notify_all();
    if (wait_for_me) cv.wait(lk, []{ return running == my_id; });
}
}
inline void vexDelay(uint32_t ms) {
    std::unique_lock<std::mutex> lk(sim::mu);
    for (auto &t : sim::thr) if (t.id == sim::my_id) t.wake = sim::t_ms + (ms == 0 ? 0.001 : ms);
    sim::handOff(lk, true);
}
namespace vex {
struct task { task(int (*fn)()) {
    std::lock_guard<std::mutex> g(sim::mu);
    int id = sim::next_id++;
    sim::thr.push_back({id, sim::t_ms});
    std::thread([fn, id]{
        sim::my_id = id;
        { std::unique_lock<std::mutex> lk(sim::mu); sim::cv.wait(lk, [id]{ return sim::running == id; }); }
        fn();
        std::unique_lock<std::mutex> lk(sim::mu);
        for (size_t i = 0; i < sim::thr.size(); i++) if (sim::thr[i].id == id) { sim::thr.erase(sim::thr.begin() + i); break; }
        sim::handOff(lk, false);
    }).detach();
} };
struct competition { void autonomous(void(*)()){} void drivercontrol(void(*)()){} };
inline void wait(double t, timeUnits u){ vexDelay((uint32_t)(u == timeUnits::msec ? t : t * 1000)); }
}
// The macros a VEXcode project's vex.h defines (see ci/vex.h), so a name that clashes with them
// fails here too, not only in the VEX SDK build
#define waitUntil(condition)                                                   \
  do {                                                                         \
    vex::wait(5, vex::msec);                                                   \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
