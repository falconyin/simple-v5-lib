#pragma once
// A tiny drivetrain simulator that stands in for the VEX SDK, so the library can be tested on a
// computer (see tests/run_tests.sh). It only has the parts of the VEX API the library uses.
//
// - Each side of the drivetrain responds to its voltage like a real motor (speed follows the
//   command with a 0.1 s delay). Ports 1-3 are the left side, 4-6 the right side.
// - The inertial sensor reports the heading worked out from the two wheel speeds.
// - Tasks are real threads, but only one runs at a time and they only switch inside vexDelay /
//   wait, like on the V5 brain. Time is simulated, so the tests run much faster than real time.
#include <cmath>
#include <cstdint>
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
const double MAXV = 70, TAU = 0.1, TRACK = 12;
inline void step() {               // 1 ms
    for (int s = 0; s < 2; s++) {
        double target = mode[s] == 0 ? cmd[s] / 100 * MAXV * gain[s] : 0;
        double tau = mode[s] == 0 ? TAU : (mode[s] == 1 ? 0.5 : 0.03);
        v[s] += (target - v[s]) * 0.001 / tau;
        dist[s] += v[s] * 0.001;
        peak = fmax(peak, fabs(v[s]));
    }
    rate = (v[0] - v[1]) / TRACK * 180 / M_PI;
    heading += rate * 0.001;
    double vc = (v[0] + v[1]) / 2;
    x += vc * sin(heading * M_PI / 180) * 0.001;
    y += vc * cos(heading * M_PI / 180) * 0.001;
    t_ms += 1;
}
}
namespace vex {
enum { PORT1=0,PORT2,PORT3,PORT4,PORT5,PORT6,PORT7 };
enum gearSetting { ratio36_1, ratio18_1, ratio6_1 };
enum class rotationUnits { deg, rev };
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
struct brain { double timer(timeUnits u){ return u == timeUnits::msec ? sim::t_ms : sim::t_ms / 1000; } screen Screen; };
struct axis { int value = 0; int position(percentUnits){ return value; } };
struct button { bool pressing(){ return false; } };
struct controller { axis Axis1, Axis2, Axis3, Axis4; button ButtonLeft, ButtonRight; screen Screen; };
const double CIRC = 3.25 * M_PI, RATIO = 2.0 / 3.0;
struct motor { int side; motor(int port, gearSetting, bool) : side(port >= 3 ? 1 : 0) {}
  double position(rotationUnits){ return sim::dist[side] / CIRC / RATIO; }
  double velocity(velocityUnits){ return sim::v[side] / CIRC / RATIO * 360; }
  void stop(brakeType b){ sim::mode[side] = b == brakeType::coast ? 1 : 2; }
  void spin(directionType, double mv, voltageUnits){ sim::mode[side] = 0; sim::cmd[side] = fmax(-100, fmin(100, mv / 120)); sim::peak_cmd = fmax(sim::peak_cmd, fabs(sim::cmd[side])); } };
struct motor_group { motor *m; template<class... M> motor_group(motor &first, M&...) : m(&first) {}
  void stop(brakeType b){ m->stop(b); } void spin(directionType d, double x, voltageUnits u){ m->spin(d, x, u); } };
struct inertial { inertial(int){} double rotation(rotationUnits){ return sim::heading; }
  double gyroRate(axisType, rateUnits){ return -sim::rate; } // counter-clockwise positive, like the real IMU (assumed)
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
