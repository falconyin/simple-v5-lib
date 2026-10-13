// Draws the diagrams in docs/images as SVG files. The path diagrams come from the simulator
// (tests/sim/vex.h): the library's own code drives those movements, and the picture shows where the
// simulated robot went. The others (coordinates, offsets, the wall) are drawn from fixed numbers.
// It also drives the guide's other examples and prints the numbers docs/odometry-and-paths.md quotes.
// The tuning graphs (docs/tuning.md) are what the Brain screen shows during a movement: every line
// the library drew on the simulator's screen, and their numbers are printed too. The autonomous
// picture (docs/writing-autonomous.md) is examples/autonomous's own routine, from both starting spots.
// The arm and intake pictures (docs/mechanisms.md) are the library's Arm and Intake on the simulator.
// Run docs/diagrams/make_diagrams.sh after changing how the robot drives, look at the pictures, and
// compare the numbers with the guides.

#include "vex.h"
#include "simpleV5lib.h"
#include <unistd.h>
#include <string>
#include <vector>

// examples/autonomous, whose routine docs/writing-autonomous.md shows: the picture drives the
// example's own code (its main() is renamed, so it doesn't clash with this one)
#define main autonomous_example_main
#include "../../examples/autonomous/main.cpp"
#undef main

// ---------- Colors (a light card, so the pictures read the same on light and dark pages) ----------
const char *SURFACE = "#fcfcfb", *INK = "#0b0b0b", *INK_2 = "#52514e", *GRID = "#e4e3df", *AXIS = "#a8a79f";
const char *BLUE = "#2a78d6", *ORANGE = "#eb6834", *AQUA = "#1baf7a", *YELLOW = "#eda100";

// A picture of part of the field: x to the right, y up the page (heading 0 points up)
struct Picture {
    double x0, y0, x1, y1; // the part of the field shown, inches
    double scale;          // pixels per inch
    double left = 40, top = 40, right = 20, bottom = 40; // room around it, pixels
    std::string body;

    Picture(double x0_, double y0_, double x1_, double y1_, double scale_)
        : x0(x0_), y0(y0_), x1(x1_), y1(y1_), scale(scale_) {}

    double X(double x) const { return left + (x - x0) * scale; }
    double Y(double y) const { return top + (y1 - y) * scale; }
    double width() const { return left + (x1 - x0) * scale + right; }
    double height() const { return top + (y1 - y0) * scale + bottom; }

    void add(const std::string &s) { body += s; body += "\n"; }
    void line(double ax, double ay, double bx, double by, const char *color, double w = 2, const char *dash = "") {
        add(fmt("<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='%.1f' stroke-linecap='round'%s/>",
                X(ax), Y(ay), X(bx), Y(by), color, w, dash));
    }
    void arrow(double ax, double ay, double bx, double by, const char *color, double w = 2) {
        line(ax, ay, bx, by, color, w);
        double angle = atan2(Y(by) - Y(ay), X(bx) - X(ax)), size = 9;
        add(fmt("<path d='M%.1f %.1f L%.1f %.1f L%.1f %.1f Z' fill='%s'/>", X(bx), Y(by),
                X(bx) - size * cos(angle - 0.4), Y(by) - size * sin(angle - 0.4),
                X(bx) - size * cos(angle + 0.4), Y(by) - size * sin(angle + 0.4), color));
    }
    // A dashed ring, r in pixels
    void circle(double x, double y, double r, const char *color) {
        add(fmt("<circle cx='%.1f' cy='%.1f' r='%.1f' fill='none' stroke='%s' stroke-width='1.2' stroke-dasharray='3 4'/>",
                X(x), Y(y), r, color));
    }
    void dot(double x, double y, double r, const char *color) {
        add(fmt("<circle cx='%.1f' cy='%.1f' r='%.1f' fill='%s' stroke='%s' stroke-width='2'/>", X(x), Y(y), r, color, SURFACE));
    }
    // anchor: start, middle or end. dx/dy move the text in pixels.
    void text(double x, double y, const std::string &s, const char *anchor = "start", const char *color = INK,
              double dx = 0, double dy = 0, int size = 13) {
        add(fmt("<text x='%.1f' y='%.1f' text-anchor='%s' fill='%s' font-size='%d'>", X(x) + dx, Y(y) + dy, anchor, color, size) + s + "</text>");
    }
    void path(const std::vector<std::pair<double, double>> &points, const char *color, double w = 2.5) {
        std::string d;
        for (auto &p : points) d += fmt("%s%.1f %.1f ", d.empty() ? "M" : "L", X(p.first), Y(p.second));
        // (d can be far longer than fmt's buffer)
        add("<path d='" + d + fmt("' fill='none' stroke='%s' stroke-width='%.1f' stroke-linejoin='round' stroke-linecap='round'/>",
                                  color, w));
    }
    // Field tiles are 24 inches: a light line every 12, a darker one on the tile edges
    void grid() {
        for (double x = ceil(x0 / 12) * 12; x <= x1; x += 12) line(x, y0, x, y1, fmod(fabs(x), 24) < 1 ? AXIS : GRID, 1);
        for (double y = ceil(y0 / 12) * 12; y <= y1; y += 12) line(x0, y, x1, y, fmod(fabs(y), 24) < 1 ? AXIS : GRID, 1);
        for (double x = ceil(x0 / 12) * 12; x <= x1; x += 12) text(x, y0, fmt("%g", x + 0.0), "middle", INK_2, 0, 18, 11);
        for (double y = ceil(y0 / 12) * 12; y <= y1; y += 12) text(x0, y, fmt("%g", y + 0.0), "end", INK_2, -6, 4, 11); // + 0.0: not "-0"
    }
    // A point on the robot, given how far it is ahead of and right of the robot's center
    static std::pair<double, double> onRobot(double x, double y, double heading, double ahead, double right) {
        double h = heading * M_PI / 180;
        return {x + ahead * sin(h) + right * cos(h), y + ahead * cos(h) - right * sin(h)};
    }
    // A robot seen from above: a square with its front edge drawn thick
    void robot(double x, double y, double heading, double size, const char *color, double opacity = 1) {
        std::string d;
        double corners[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
        for (auto &c : corners) {
            auto p = onRobot(x, y, heading, c[0] * size / 2, c[1] * size / 2);
            d += fmt("%s%.1f %.1f ", d.empty() ? "M" : "L", X(p.first), Y(p.second));
        }
        add(fmt("<path d='%sZ' fill='%s' fill-opacity='%.2f' stroke='%s' stroke-opacity='%.2f' stroke-width='1.5'/>",
                d.c_str(), color, 0.08 * opacity, color, opacity));
        auto a = onRobot(x, y, heading, size / 2, -size / 2), b = onRobot(x, y, heading, size / 2, size / 2);
        add(fmt("<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-opacity='%.2f' stroke-width='4' stroke-linecap='round'/>",
                X(a.first), Y(a.second), X(b.first), Y(b.second), color, opacity));
    }
    // Where the robot ends up, and which way it faces then
    void pose(double x, double y, double heading, const char *color, double length = 5) {
        auto tip = onRobot(x, y, heading, length, 0);
        arrow(x, y, tip.first, tip.second, color, 2);
        dot(x, y, 4.5, color);
    }

    void save(const std::string &folder, const char *name, const char *title) const {
        std::string file = folder + "/" + name;
        FILE *f = fopen(file.c_str(), "w");
        if (!f) { printf("can't write %s\n", file.c_str()); _exit(1); }
        fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f' viewBox='0 0 %.0f %.0f' "
                   "font-family='-apple-system, Segoe UI, Helvetica, Arial, sans-serif'>\n<title>%s</title>\n"
                   "<rect width='100%%' height='100%%' rx='8' fill='%s'/>\n%s</svg>\n",
                width(), height(), width(), height(), title, SURFACE, body.c_str());
        fclose(f);
        printf("wrote %s\n", file.c_str());
    }

    template <typename... T> static std::string fmt(const char *format, T... values) {
        char buffer[4096];
        snprintf(buffer, sizeof buffer, format, values...);
        return buffer;
    }
};

// ---------- Recording where the simulated robot goes ----------

struct Sample { double t, x, y, heading; };
static std::vector<Sample> recorded;
static bool recording = false;

static int recorder() {
    while (true) {
        if (recording) recorded.push_back({sim::t_ms, sim::x, sim::y, getInertial()});
        vexDelay(10);
    }
    return 0;
}

// Stop, and put the robot at (0, 0) facing 0 (like the tests' reset), extra_ms later than usual
static void startAtOrigin(int extra_ms = 0) {
    stopDriving();
    vexDelay(500 + extra_ms);
    sim::x = 0;
    sim::y = 0;
    sim::forgetReadings();
    setPose(0, 0, 0);
    recorded.clear();
    recording = true;
}

static std::vector<std::pair<double, double>> track() {
    recording = false;
    std::vector<std::pair<double, double>> points;
    for (auto &s : recorded) points.push_back({s.x, s.y});
    return points;
}

// ---------- The diagrams ----------

// The field's coordinates: where (0, 0) is, which way x, y and the heading go
static void fieldCoordinates(const std::string &folder) {
    Picture p{-36, -24, 48, 48, 6};
    p.grid();
    p.arrow(-36, 0, 46, 0, INK, 1.5);
    p.arrow(0, -24, 0, 46, INK, 1.5);
    p.text(46, 0, "x", "end", INK, 0, -8, 15);
    p.text(0, 46, "y", "start", INK, 8, 4, 15);
    p.dot(0, 0, 4, INK);
    p.text(0, 0, "(0, 0)", "start", INK, 6, 16);

    // A robot at (24, 24), facing 60 degrees
    p.robot(24, 24, 60, 15, BLUE);
    p.pose(24, 24, 60, BLUE, 12);
    p.text(24, 12, "x = 24, y = 24, heading 60", "middle", INK, 0, 4);

    // The compass: 0 up, clockwise positive
    double cx = -20, cy = 30, r = 9;
    p.add(Picture::fmt("<circle cx='%.1f' cy='%.1f' r='%.1f' fill='none' stroke='%s' stroke-width='1.5'/>",
                       p.X(cx), p.Y(cy), r * p.scale, AXIS));
    const char *labels[4] = {"0", "90", "180", "270"};
    for (int i = 0; i < 4; i++) {
        double h = i * M_PI / 2;
        p.line(cx + (r - 1.5) * sin(h), cy + (r - 1.5) * cos(h), cx + (r + 1.5) * sin(h), cy + (r + 1.5) * cos(h), INK, 1.5);
        p.text(cx + (r + 4) * sin(h), cy + (r + 4) * cos(h), labels[i], "middle", INK, 0, 5);
    }
    // A clockwise arrow from about 20 to about 70 degrees
    std::string d;
    for (int a = 20; a <= 70; a += 5) {
        double h = a * M_PI / 180;
        d += Picture::fmt("%s%.1f %.1f ", d.empty() ? "M" : "L", p.X(cx + (r - 3) * sin(h)), p.Y(cy + (r - 3) * cos(h)));
    }
    p.add(Picture::fmt("<path d='%s' fill='none' stroke='%s' stroke-width='2'/>", d.c_str(), ORANGE));
    double end = 70 * M_PI / 180, before = 62 * M_PI / 180;
    p.arrow(cx + (r - 3) * sin(before), cy + (r - 3) * cos(before), cx + (r - 3) * sin(end), cy + (r - 3) * cos(end), ORANGE);
    p.text(cx, cy - r - 4, "heading: clockwise", "middle", INK_2, 0, 30, 12);
    p.save(folder, "field-coordinates.svg", "Field coordinates: x to the right, y forward, heading clockwise from +y");
}

// Where things sit on the robot: the offsets in simpleV5LibConfig.h, all measured from the center
static void robotOffsets(const std::string &folder) {
    Picture p{-12, -12, 34, 13, 14};
    p.top = 30;
    p.bottom = 20;
    // The robot at heading 0, 15 inches square, with its drive wheels
    p.robot(0, 0, 0, 15, INK_2);
    for (double side : {-1.0, 1.0})
        for (double ahead : {-5.0, 0.0, 5.0})
            p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='3' fill='%s'/>",
                               p.X(side * 6.5 - 0.6), p.Y(ahead + 1.6), 1.2 * p.scale, 3.2 * p.scale, AXIS));
    p.text(0, 7.5, "front", "middle", INK_2, 0, -10);
    p.dot(0, 0, 4.5, INK);
    p.text(0, 0, "center", "middle", INK, 0, 20);

    // Forward tracking wheel, 3 inches right of the center: rolls when the robot drives forward
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='3' fill='%s'/>",
                       p.X(3 - 0.4), p.Y(1.4), 0.8 * p.scale, 2.8 * p.scale, BLUE));
    p.line(0, -1.8, 3, -1.8, BLUE, 1.5, " stroke-dasharray='3 3'");
    // Sideways tracking wheel, 4 inches behind the center: rolls when the robot is pushed sideways
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='3' fill='%s'/>",
                       p.X(-1.4), p.Y(-4 + 0.4), 2.8 * p.scale, 0.8 * p.scale, ORANGE));
    p.line(-2.2, 0, -2.2, -4, ORANGE, 1.5, " stroke-dasharray='3 3'");
    // Front distance sensor: its front (where the beam starts) 6 inches ahead, 2 to the right
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='2' fill='%s'/>",
                       p.X(2 - 0.8), p.Y(6), 1.6 * p.scale, 1.0 * p.scale, AQUA));
    p.line(0, 6, 2, 6, AQUA, 1.5, " stroke-dasharray='3 3'");
    p.line(-1, 0, -1, 6, AQUA, 1.5, " stroke-dasharray='3 3'");
    p.arrow(2, 6, 2, 11.5, AQUA, 1.5);

    // The labels, to the right of the robot
    double lx = 11;
    p.text(lx, 9, "<tspan font-weight='600'>Front distance sensor</tspan>");
    p.text(lx, 9, "DISTANCE_FRONT_AHEAD = 6   (ahead: +)", "start", INK_2, 0, 18);
    p.text(lx, 9, "DISTANCE_FRONT_RIGHT = 2   (right: +)", "start", INK_2, 0, 36);
    p.text(lx, 9, "measured to the sensor's front, where the beam starts", "start", INK_2, 0, 54, 12);
    p.line(2.9, 5.5, lx - 0.5, 8.6, AQUA, 1);
    p.text(lx, 1, "<tspan font-weight='600'>Forward tracking wheel</tspan>");
    p.text(lx, 1, "TRACKING_FORWARD_OFFSET = 3   (right: +)", "start", INK_2, 0, 18);
    p.line(3.5, 0.5, lx - 0.5, 0.6, BLUE, 1);
    p.text(lx, -6, "<tspan font-weight='600'>Sideways tracking wheel</tspan>");
    p.text(lx, -6, "TRACKING_SIDEWAYS_OFFSET = -4   (behind: -)", "start", INK_2, 0, 18);
    p.line(1.5, -4, lx - 0.5, -6.4, ORANGE, 1);
    p.save(folder, "robot-offsets.svg", "Where the tracking wheels and distance sensors sit, measured from the robot's center");
}

// resetXFromWall: from the wall, back along the beam to the sensor, then to the robot's center
static void wallReset(const std::string &folder) {
    Picture p{30, -4, 78, 26, 12};
    p.right = 30;
    double wall = 70, x = 48, y = 10, heading = 80, ahead = 6, right = 2;
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='%s'/>",
                       p.X(wall), p.Y(26), (p.x1 - wall) * p.scale, 30 * p.scale, GRID));
    p.line(wall, -4, wall, 26, INK, 3);
    p.text(wall, 24, Picture::fmt("wall at x = %g", wall), "start", INK, 8, 4);

    p.robot(x, y, heading, 15, INK_2);
    p.dot(x, y, 4.5, INK);
    p.text(x, y, "robot center", "middle", INK, 0, -12);
    auto sensor = Picture::onRobot(x, y, heading, ahead, right);
    p.line(x, y, sensor.first, sensor.second, AQUA, 2, " stroke-dasharray='4 3'");
    p.text((x + sensor.first) / 2, (y + sensor.second) / 2, "sensor offset", "middle", INK_2, 0, 22, 12);

    // The beam, along the sensor's direction, to the wall
    double h = heading * M_PI / 180, reading = (wall - sensor.first) / sin(h);
    double hit_y = sensor.second + reading * cos(h);
    p.arrow(sensor.first, sensor.second, wall, hit_y, AQUA, 2.5);
    p.dot(sensor.first, sensor.second, 4.5, AQUA);
    p.text((sensor.first + wall) / 2, (sensor.second + hit_y) / 2, Picture::fmt("reading = %.1f in", reading), "middle", INK, 0, -12);
    // Straight on would be along x: the angle between them must be under DISTANCE_RESET_MAX_ANGLE
    p.line(sensor.first, sensor.second, wall, sensor.second, ORANGE, 1.5, " stroke-dasharray='4 4'");
    p.text(wall, sensor.second, Picture::fmt("%g° off straight (at most %g)", fabs(90 - heading), DISTANCE_RESET_MAX_ANGLE),
           "end", ORANGE, -8, 18, 12);

    p.text(30, -4, Picture::fmt("x = %g - reading × sin(%g°) - sensor offset in x", wall, heading), "start", INK, 0, 30, 13);
    p.bottom = 50;
    p.save(folder, "wall-reset.svg", "resetXFromWall: the wall's x, back along the beam, then from the sensor to the robot's center");
}

// In src/simpleV5lib.cpp: where PID_drive_to_pose's carrot is, for a robot at (robot_x, robot_y)
void poseCarrot(double robot_x, double robot_y, double target_x, double target_y, double target_heading,
                double lead, bool backwards, double &x, double &y);

// The carrot for a robot at (x, y) driving forward to the pose (tx, ty, th). Returns whether the
// look-ahead moved it (it is closer to the target than lead × distance).
static bool carrot(double x, double y, double tx, double ty, double th, double lead, double &cx, double &cy) {
    poseCarrot(x, y, tx, ty, th, lead, false, cx, cy);
    return hypot(tx - cx, ty - cy) < lead * hypot(tx - x, ty - y) - 0.01;
}

// PID_drive_to_pose: the carrot point the robot steers towards, at a few moments of the drive
static void boomerang(const std::string &folder) {
    double tx = 24, ty = 36, th = 90, lead = POSE_LEAD;
    startAtOrigin();
    PID_drive_to_pose(tx, ty, th, 0.5, 0.2);
    auto points = track();
    Picture p{-18, -6, 44, 42, 9};
    p.grid();
    // The line the robot must arrive along
    p.line(tx - 40, ty, tx, ty, ORANGE, 1.5, " stroke-dasharray='2 5'");
    p.path(points, BLUE);
    // The carrot at a few moments: lead × distance left, behind the target along its heading, or
    // pulled forward to the look-ahead circle near the line. The robot and its carrot at the same
    // moment get the same number.
    int moment = 0;
    for (double left : {40.0, 26.0, 16.0}) {
        for (auto &s : recorded) {
            double d = hypot(tx - s.x, ty - s.y);
            if (d > left) continue;
            double cx, cy;
            if (carrot(s.x, s.y, tx, ty, th, lead, cx, cy))
                p.circle(s.x, s.y, 2 * POINT_AIM_DISTANCE * p.scale, ORANGE);
            moment++;
            p.line(s.x, s.y, cx, cy, INK_2, 1.2, " stroke-dasharray='4 3'");
            p.dot(s.x, s.y, 4, BLUE);
            p.dot(cx, cy, 5, ORANGE);
            p.text(s.x, s.y, Picture::fmt("%d", moment), "end", BLUE, -8, 4, 13);
            p.text(cx, cy, Picture::fmt("%d", moment), "middle", ORANGE, 0, -10, 13);
            break;
        }
    }
    // Legend, bottom right
    p.dot(8, 6, 4, BLUE);
    p.text(9.5, 6, "the robot at moments 1, 2, 3", "start", INK, 0, 4, 12);
    p.dot(8, 3.4, 5, ORANGE);
    p.text(9.5, 3.4, "its carrot then (dashed: where it steers)", "start", INK, 0, 4, 12);
    p.circle(8, 0.8, 6, ORANGE);
    p.text(9.5, 0.8, Picture::fmt("near the line: the carrot stays %g in ahead", 2 * POINT_AIM_DISTANCE), "start", INK, 0, 4, 12);
    p.pose(0, 0, 0, INK);
    p.text(0, 0, "start", "start", INK_2, 8, 16);
    p.pose(tx, ty, th, INK);
    p.text(tx, ty, "target (24, 36), heading 90", "start", INK, 6, 24);
    p.save(folder, "pose-carrot.svg", "PID_drive_to_pose steers towards a carrot that slides onto the target as the robot gets closer");
}

// PID_drive_to_pose with different leads: wider curves line up with the heading earlier
static void poseLeads(const std::string &folder) {
    double tx = 24, ty = 36, th = 90;
    Picture p{-18, -6, 36, 42, 9};
    p.grid();
    struct { double lead; const char *color; } runs[] = {{0, YELLOW}, {0.3, AQUA}, {0.6, BLUE}, {1, ORANGE}};
    double ly = 12;
    for (auto &r : runs) {
        startAtOrigin();
        PID_drive_to_pose(tx, ty, th, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, false, r.lead);
        auto points = track();
        p.path(points, r.color);
        // Legend, bottom right
        p.line(16, ly, 19, ly, r.color, 3);
        p.text(20, ly, Picture::fmt("lead %g%s", r.lead, r.lead == POSE_LEAD ? " (POSE_LEAD)" : ""), "start", INK, 0, 4, 12);
        ly -= 2.6;
    }
    p.pose(0, 0, 0, INK);
    p.text(0, 0, "start", "start", INK_2, 8, 16);
    p.pose(tx, ty, th, INK);
    p.text(tx, ty, "target, heading 90", "end", INK, 0, -14);
    p.save(folder, "pose-lead.svg", "PID_drive_to_pose(24, 36, 90) with lead 0, 0.3, 0.6 and 1");
}

// The same four corners, driven one by one and as a chain
static void chainedSquare(const std::string &folder) {
    Picture p{-10, -6, 74, 34, 8};
    p.top = 50;
    double shift = 40; // the chained square is drawn 40 inches further right
    for (double x : {0.0, 24.0})
        for (double y : {0.0, 24.0}) {
            p.dot(x, y, 4, INK_2);
            p.dot(x + shift, y, 4, INK_2);
        }

    startAtOrigin();
    PID_drive_to_point(0, 24, 0.5, 0.2);
    PID_drive_to_point(24, 24, 0.5, 0.2);
    PID_drive_to_point(24, 0, 0.5, 0.2);
    PID_drive_to_point(0, 0, 0.5, 0.2);
    double stop_time = recorded.back().t - recorded.front().t;
    p.path(track(), BLUE);

    startAtOrigin();
    PID_drive_to_point_chain(0, 24, 2);
    PID_drive_to_point_chain(24, 24, 2);
    PID_drive_to_point_chain(24, 0, 2);
    PID_drive_to_point(0, 0, 0.5, 0.2);
    double chain_time = recorded.back().t - recorded.front().t;
    auto points = track();
    for (auto &q : points) q.first += shift;
    p.path(points, ORANGE);

    p.text(12, 34, "<tspan font-weight='600'>PID_drive_to_point × 4</tspan>", "middle", INK, 0, -24);
    p.text(12, 34, Picture::fmt("stops and turns at every corner: %.1f s", stop_time / 1000), "middle", INK_2, 0, -6, 12);
    p.text(12 + shift, 34, "<tspan font-weight='600'>3 × _chain (exit_range 2), then PID_drive_to_point</tspan>", "middle", INK, 0, -24);
    p.text(12 + shift, 34, Picture::fmt("curves through the corners: %.1f s", chain_time / 1000), "middle", INK_2, 0, -6, 12);
    p.text(0, 0, "start / end", "middle", INK_2, 0, 20, 11);
    p.text(shift, 0, "start / end", "middle", INK_2, 0, 20, 11);
    p.save(folder, "chained-points.svg", "Four corners driven one by one, and the same corners as a chain");
}

// ---------- The numbers in docs/odometry-and-paths.md ----------
// Printed, not drawn: after a change, compare them with the guide's text and tables.
//
// Each one is driven STARTS times, starting 0, 1, 2, ... ms later. The library's loops run every 10 ms,
// and where the start falls between their ticks changes the result a little, and at some corners a
// lot (a chain can hand over one tick earlier or later). So the guide quotes the range, not one run.
const int STARTS = 10;

// The smallest and biggest of a number over the runs
struct Range {
    double low = 1e9, high = -1e9;
    void add(double value) { low = fmin(low, value); high = fmax(high, value); }
    std::string show(const char *format = "%.1f") const {
        std::string a = Picture::fmt(format, low), b = Picture::fmt(format, high);
        return a == b ? a : a + " to " + b;
    }
};

static double timeTaken() { return (recorded.back().t - recorded.front().t) / 1000; }

// How close the robot came to (x, y)
static double closestTo(double x, double y) {
    double closest = 1e9;
    for (auto &s : recorded) closest = fmin(closest, hypot(s.x - x, s.y - y));
    return closest;
}

// How far the robot got from the straight line a -> b, after it came closest to a (the corner)
static double swingOut(double ax, double ay, double bx, double by) {
    size_t corner = 0;
    for (size_t i = 0; i < recorded.size(); i++)
        if (hypot(recorded[i].x - ax, recorded[i].y - ay) < hypot(recorded[corner].x - ax, recorded[corner].y - ay)) corner = i;
    double length = hypot(bx - ax, by - ay), along_x = (bx - ax) / length, along_y = (by - ay) / length, most = 0;
    for (size_t i = corner; i < recorded.size(); i++)
        most = fmax(most, fabs((recorded[i].x - ax) * along_y - (recorded[i].y - ay) * along_x));
    return most;
}

static void numbers() {
    printf("\nEach number: the smallest to the biggest of %d runs, started 0 to %d ms apart\n", STARTS, STARTS - 1);
    printf("\nSection 4, PID_drive_to_pose(24, 36, 90) from (0, 0, 0): how far from the target it ends\n");
    for (double lead : {0.0, 0.3, 0.6, 1.0}) {
        Range miss, heading_off;
        for (int start = 0; start < STARTS; start++) {
            startAtOrigin(start);
            PID_drive_to_pose(24, 36, 90, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, false, lead);
            recording = false;
            miss.add(hypot(sim::x - 24, sim::y - 36));
            heading_off.add(fabs(getInertial() - 90));
        }
        printf("  lead %g: %s in, heading %s degrees off\n", lead, miss.show().c_str(), heading_off.show().c_str());
    }
    Range short_miss;
    for (int start = 0; start < STARTS; start++) {
        startAtOrigin(start);
        PID_drive_to_pose(10, 10, 0, 0.5, 0.2);
        recording = false;
        short_miss.add(hypot(sim::x - 10, sim::y - 10));
    }
    printf("  the short pose (10, 10, 0): %s in\n", short_miss.show().c_str());

    printf("\nSection 5, the exit_range table: three points from (0, 0, 0), the corner at the first one\n");
    struct { const char *name; double points[3][2]; } paths[] = {
        {"gentle", {{0, 24}, {10, 48}, {20, 72}}},
        {"sharp", {{0, 24}, {24, 24}, {48, 24}}},
    };
    for (auto &path : paths) {
        auto &p = path.points;
        Range stopping;
        for (int start = 0; start < STARTS; start++) {
            startAtOrigin(start);
            for (auto &point : p) PID_drive_to_point(point[0], point[1], 0.5, 0.2);
            recording = false;
            stopping.add(timeTaken());
        }
        printf("  %s, normal movements: %s s\n", path.name, stopping.show().c_str());
        for (double exit_range : {2.0, 4.0, 8.0}) {
            Range time, swing, corner;
            for (int start = 0; start < STARTS; start++) {
                startAtOrigin(start);
                PID_drive_to_point_chain(p[0][0], p[0][1], exit_range);
                PID_drive_to_point_chain(p[1][0], p[1][1], exit_range);
                PID_drive_to_point(p[2][0], p[2][1], 0.5, 0.2);
                recording = false;
                time.add(timeTaken());
                swing.add(swingOut(p[0][0], p[0][1], p[2][0], p[2][1]));
                corner.add(closestTo(p[0][0], p[0][1]));
            }
            printf("  %s, exit_range %g: %s s, swings out %s in, passes %s in from the corner\n", path.name, exit_range,
                   time.show().c_str(), swing.show().c_str(), corner.show().c_str());
        }
    }

    printf("\nSection 5, the square in chained-points.svg\n");
    Range stopping, chained, faster, corners;
    for (int start = 0; start < STARTS; start++) {
        startAtOrigin(start);
        PID_drive_to_point(0, 24, 0.5, 0.2);
        PID_drive_to_point(24, 24, 0.5, 0.2);
        PID_drive_to_point(24, 0, 0.5, 0.2);
        PID_drive_to_point(0, 0, 0.5, 0.2);
        recording = false;
        double stop_time = timeTaken();
        startAtOrigin(start);
        PID_drive_to_point_chain(0, 24, 2);
        PID_drive_to_point_chain(24, 24, 2);
        PID_drive_to_point_chain(24, 0, 2);
        PID_drive_to_point(0, 0, 0.5, 0.2);
        recording = false;
        stopping.add(stop_time);
        chained.add(timeTaken());
        faster.add(100 * (1 - timeTaken() / stop_time));
        for (auto &c : {std::make_pair(0, 24), std::make_pair(24, 24), std::make_pair(24, 0)}) corners.add(closestTo(c.first, c.second));
    }
    printf("  stopping: %s s, chained: %s s (%s%% less), passes %s in from the corners\n", stopping.show().c_str(),
           chained.show().c_str(), faster.show("%.0f").c_str(), corners.show().c_str());
}

// examples/autonomous's routine from both starting spots, which are mirror images of each other:
// the picture (each robot counts from where it starts, so each path is drawn from its own spot), and
// the numbers docs/writing-autonomous.md quotes. Each spot is driven STARTS times; the picture is the
// first run. The path comes from the example's own code, but the places below are copies of the
// example's points: change them together.
static void autonomousExample(const std::string &folder) {
    const double PIECE_Y = 30, GOAL_X = 24, GOAL_Y = 48, BACKED_X = 14, PARK_X = -12, PARK_Y = 6;
    Picture p{-62, -10, 62, 58, 5.5};
    p.top = 56;
    // Tile lines without numbers: every robot has its own (0, 0)
    for (double x = -48; x <= 48; x += 24) p.line(x, p.y0, x, p.y1, GRID, 1);
    for (double y = 0; y <= 48; y += 24) p.line(p.x0, y, p.x1, y, GRID, 1);
    p.line(0, p.y0, 0, p.y1, AXIS, 1.5, " stroke-dasharray='6 5'");
    p.text(0, p.y0, "the mirror line", "middle", INK_2, 0, 18, 11);
    std::string summary; // printed after all the runs, so the example's own "Autonomous took" lines come first
    struct { bool mirrored; const char *color; } runs[] = {{false, BLUE}, {true, ORANGE}};
    for (auto &r : runs) {
        double side = r.mirrored ? -1 : 1, shift = -40 * side; // the left spot is 40 inches left of the line
        Range time, goal, end, heading;
        for (int start = 0; start < STARTS; start++) {
            startAtOrigin(start);
            double started = sim::t_ms; // timed like the example times itself, with the Brain's timer
            if (r.mirrored) rightSpot(); else leftSpot();
            time.add((sim::t_ms - started) / 1000);
            goal.add(closestTo(GOAL_X * side, GOAL_Y));
            end.add(hypot(sim::x - PARK_X * side, sim::y - PARK_Y));
            heading.add(fabs(remainder(getInertial(), 360)));
            auto points = track();
            if (start > 0) continue;
            for (auto &q : points) q.first += shift;
            p.robot(shift, 0, 0, 14, INK, 0.6);
            p.path(points, r.color);
            p.pose(sim::x + shift, sim::y, getInertial(), r.color);
        }
        summary += Picture::fmt("  %s: %s s, passes %s in from the goal, ends %s in from (%g, %g), heading %s degrees off\n",
                                r.mirrored ? "rightSpot" : "leftSpot", time.show().c_str(), goal.show().c_str(),
                                end.show().c_str(), PARK_X * side, PARK_Y, heading.show().c_str());
        // Where each step of the routine goes, numbered like the comments in the example
        p.dot(shift, PIECE_Y, 5, YELLOW);
        p.text(shift, PIECE_Y, "1", r.mirrored ? "end" : "start", INK, 9 * side, 4, 13);
        p.dot(shift + GOAL_X * side, GOAL_Y, 5, r.color);
        p.text(shift + GOAL_X * side, GOAL_Y, "2, 3", "middle", r.color, 0, -12, 13);
        p.text(shift + BACKED_X * side, GOAL_Y, "4", "middle", r.color, 0, 20, 13);
        p.text(shift + PARK_X * side, PARK_Y, "5", r.mirrored ? "start" : "end", r.color, -10 * side, 4, 13);
        p.text(shift, 0, "start (0, 0)", "middle", INK_2, 0, 24, 11);
        p.text(shift, p.y1, r.mirrored ? "<tspan font-weight='600'>right spot: mirrored = true</tspan>"
                                       : "<tspan font-weight='600'>left spot: mirrored = false</tspan>",
               "middle", r.color, 0, -30);
        p.text(shift, p.y1, Picture::fmt("goal at (%g, %g) from its start, %s s", GOAL_X * side, GOAL_Y, time.show().c_str()),
               "middle", INK_2, 0, -12, 12);
    }
    p.save(folder, "autonomous-mirrored.svg", "The example routine from the left starting spot, and mirrored from the right one");
    printf("\ndocs/writing-autonomous.md, examples/autonomous from both spots, %d runs each\n%s", STARTS, summary.c_str());
}

// ---------- The tuning graphs and numbers in docs/tuning.md ----------

// In src/controllerTuner.cpp: the line the controller shows after a try in tuneWithController
void tunerResultLine(const MovementResult &result, char *line, int size);

static std::string controllerLine(const MovementResult &result) {
    char line[64];
    tunerResultLine(result, line, sizeof(line));
    return line;
}

// The Brain screen after a movement with graphOnScreen(true): every line and text src/telemetry.cpp
// drew on the simulator's screen (480 x 240, y 0 at the top), in the same places and colors, with a
// time scale and the controller's line underneath.
static void brainScreen(const std::string &folder, const char *file, const char *title,
                        const std::vector<vex::drawing> &drawn, double timeout_ms, const MovementResult &result) {
    const char *SCREEN = "#000000", *PENS[] = {"#ffffff", "#ff3b30", "#30d158", "#ffffff"}; // white, red, green, white
    double pad = 16, w = 480, h = 240;
    Picture p{0, 0, w, h, 1};
    p.left = pad; p.top = pad; p.right = pad; p.bottom = 64;
    auto X = [&](double x) { return pad + x; };
    auto Y = [&](double y) { return pad + y; };
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.0f' height='%.0f' rx='4' fill='%s'/>", X(0), Y(0), w, h, SCREEN));

    // The lines, joined into one path per color where one starts at the end of the one before
    std::string paths[4];
    int end_x[4] = {-1, -1, -1, -1}, end_y[4] = {-1, -1, -1, -1};
    for (auto &d : drawn) {
        int pen = d.pen >= 0 && d.pen < 4 ? d.pen : 0;
        if (!d.text.empty()) {
            p.add(Picture::fmt("<text x='%.1f' y='%.1f' fill='%s' font-size='14'>", X(d.x1), Y(d.y1), PENS[pen]) + d.text + "</text>");
            continue;
        }
        if (d.x1 != end_x[pen] || d.y1 != end_y[pen]) paths[pen] += Picture::fmt("M%.0f %.0f ", X(d.x1), Y(d.y1));
        paths[pen] += Picture::fmt("L%.0f %.0f ", X(d.x2), Y(d.y2));
        end_x[pen] = d.x2;
        end_y[pen] = d.y2;
    }
    for (int pen : {3, 1, 2}) // the 0 line, then the error, then the power, like each tick draws them
        if (!paths[pen].empty())
            p.add("<path d='" + paths[pen] + Picture::fmt("' fill='none' stroke='%s' stroke-width='%s' stroke-linejoin='round'/>",
                                                          PENS[pen], pen == 3 ? "1" : "1.5"));

    // Under the screen: seconds, and what the controller shows
    for (double s = 0; s <= timeout_ms / 1000 + 0.01; s += 0.5) {
        double x = X(s * 1000 / timeout_ms * w);
        p.add(Picture::fmt("<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='1'/>", x, Y(h), x, Y(h + 5), AXIS));
        if (fmod(s, 1) < 0.01)
            p.add(Picture::fmt("<text x='%.1f' y='%.1f' text-anchor='middle' fill='%s' font-size='11'>%g s</text>", x, Y(h + 18), INK_2, s));
    }
    p.add(Picture::fmt("<text x='%.1f' y='%.1f' fill='%s' font-size='13'>controller: </text>", X(0), Y(h + 46), INK_2));
    p.add(Picture::fmt("<text x='%.1f' y='%.1f' fill='%s' font-size='13' font-family='Consolas, Menlo, monospace'>",
                       X(78), Y(h + 46), INK) + controllerLine(result) + "</text>");
    p.save(folder, file, title);
}

// The gains, and the friction the simulator is set to, for one example
struct Example {
    std::string file;     // the picture, or "" for a number only
    std::string title;
    bool turn;            // PID_turn(90, 0.5, 0.2), or PID_forward(24, 0.3, 0.2): what the tuner tries
    double kp, ki, kd;
    double friction = 0;  // % of power the wheels need before they move (sim::real::friction)
    double size = 0;      // degrees or inches, 0 = the tuner's 90 or 24
};

static MovementResult runExample(const Example &e, int start_ms = 0) {
    PIDGains turn = turnGains, forward = forwardGains;
    startAtOrigin(start_ms);
    recording = false;
    sim::real::friction = e.friction;
    if (e.turn) {
        turnGains = {e.kp, e.ki, e.kd};
        PID_turn(e.size ? e.size : 90, 0.5, 0.2);
    } else {
        forwardGains = {e.kp, e.ki, e.kd};
        PID_forward(e.size ? e.size : 24, 0.3, 0.2);
    }
    MovementResult result = lastMovementResult();
    sim::real::friction = 0;
    turnGains = turn;
    forwardGains = forward;
    return result;
}

// The pictures' gains: the config's turn gains, and the mistakes tuning.md shows next to them
static std::vector<Example> tuningPictures() {
    using F = Picture;
    return {
        {"tune-p-only.svg", F::fmt("PID_turn(90) with kP %g and no kD: swings past and back", TURN_KP), true, TURN_KP, 0, 0},
        {"tune-p-too-big.svg", "PID_turn(90) with kP 10 and no kD: swings until the timeout", true, 10, 0, 0},
        {"tune-pd.svg", F::fmt("PID_turn(90) with kP %g and kD %g: gets there and stops", TURN_KP, TURN_KD), true,
         TURN_KP, 0, TURN_KD},
        {"tune-d-too-big.svg", F::fmt("PID_turn(90) with kP %g and kD 100: creeps in slowly", TURN_KP), true, TURN_KP, 0, 100},
        {"tune-stops-short.svg", "PID_forward(24) with kP 6 and no kI, on wheels with friction: stops short", false, 6, 0,
         FORWARD_KD, 8},
        {"tune-with-ki.svg", "The same with kI 0.1: the I part pushes it the rest of the way", false, 6, 0.1, FORWARD_KD, 8},
    };
}

static void tuningGraphs(const std::string &folder) {
    for (auto &e : tuningPictures()) {
        graphOnScreen(true);
        MovementResult result = runExample(e);
        graphOnScreen(false);
        brainScreen(folder, e.file.c_str(), e.title.c_str(), Brain.Screen.drawn,
                    e.turn ? TURN_TIMEOUT_MS : FORWARD_TIMEOUT_MS, result);
    }
}

// The same example started STARTS times, 0 to STARTS - 1 ms apart: the controller's numbers, as ranges.
// The error keeps its sign: + = stopped short of the target, - = went past it. A picture with the same
// example is named after the numbers, and crossed off the list of pictures still to print.
static void printExample(std::string label, const Example &e, std::vector<Example> &pictures) {
    for (size_t i = 0; i < pictures.size(); i++) {
        const Example &p = pictures[i];
        if (p.turn == e.turn && p.kp == e.kp && p.ki == e.ki && p.kd == e.kd && p.friction == e.friction && p.size == e.size) {
            label += " (" + p.file + ")";
            pictures.erase(pictures.begin() + i);
            break;
        }
    }
    Range time, overshoot, error;
    int timeouts = 0;
    for (int start = 0; start < STARTS; start++) {
        MovementResult r = runExample(e, start);
        time.add(r.time_ms / 1000);
        overshoot.add(r.overshoot);
        error.add(r.error);
        timeouts += r.timed_out;
    }
    printf("  %-40s %s s, overshoot %s, ends %s%s\n", label.c_str(), time.show("%.2f").c_str(), overshoot.show("%.2f").c_str(),
           error.show("%.2f").c_str(), timeouts ? Picture::fmt(", %d of %d time out", timeouts, STARTS).c_str() : "");
}

static void tuningNumbers() {
    std::vector<Example> pictures = tuningPictures(); // the ones no table below prints
    printf("\ndocs/tuning.md: the controller's numbers, the smallest to the biggest of %d runs\n", STARTS);
    printf("(ends: + = stopped short of the target, - = went past it)\n");
    printf("The kD table, PID_turn(90) with kP %g, no kI:\n", TURN_KP);
    for (double kd : {0.0, 10.0, 20.0, TURN_KD, 60.0, 100.0})
        printExample(Picture::fmt("kD %g", kd), {"", "", true, TURN_KP, 0, kd}, pictures);
    printf("Other sizes, the config's gains:\n");
    for (double degrees : {30.0, 90.0, 180.0})
        printExample(Picture::fmt("PID_turn(%g)", degrees), {"", "", true, TURN_KP, TURN_KI, TURN_KD, 0, degrees}, pictures);
    for (double inches : {6.0, 24.0, 48.0})
        printExample(Picture::fmt("PID_forward(%g)", inches), {"", "", false, FORWARD_KP, FORWARD_KI, FORWARD_KD, 0, inches},
                     pictures);
    printf("Stopping short, PID_forward(24) with kD %g and friction 8%% (FORWARD_INTEGRAL_RANGE %g):\n", FORWARD_KD,
           FORWARD_INTEGRAL_RANGE);
    for (double kp : {4.0, 6.0})
        for (double ki : {0.0, 0.05, 0.1, 0.2})
            printExample(Picture::fmt("kP %g, kI %g", kp, ki), {"", "", false, kp, ki, FORWARD_KD, 8}, pictures);
    printf("The other pictures:\n");
    std::vector<Example> rest = pictures;
    for (auto &e : rest) printExample(Picture::fmt("kP %g, kI %g, kD %g", e.kp, e.ki, e.kd), e, pictures);
}

// ---------- The speed profile in docs/tuning.md, section 12 ----------

// One PID_forward, its true speed every 10 ms (in/s)
struct SpeedTrace {
    std::vector<double> speeds;
    MovementResult result;
};
static SpeedTrace driveAndWatch(double inches, int start_ms) {
    SpeedTrace trace;
    startAtOrigin(start_ms);
    recording = false;
    PID_forward_async(inches, 0.3, 0.2);
    while (isMoving()) {
        trace.speeds.push_back((sim::v[0] + sim::v[1]) / 2);
        vexDelay(10);
    }
    trace.result = lastMovementResult();
    return trace;
}

// The table rows: time, overshoot and end error over STARTS runs, without and with the profile
static void profileRow(const char *label, double inches, double acceleration) {
    Range time, overshoot, error;
    for (int start = 0; start < STARTS; start++) {
        speedProfile.acceleration = acceleration;
        MovementResult r = driveAndWatch(inches, start).result;
        time.add(r.time_ms / 1000);
        overshoot.add(r.overshoot);
        error.add(r.error);
    }
    speedProfile.acceleration = DRIVE_ACCELERATION;
    printf("  %-35s %s s, overshoot %s, ends %s\n", label, time.show("%.2f").c_str(), overshoot.show("%.2f").c_str(),
           error.show("%.2f").c_str());
}

static void speedProfileNumbers(const char *robot) {
    // What a user does first: measureTopSpeed, and its numbers into the config
    TopSpeedResult measured = measureTopSpeed();
    speedProfile.top_speed = measured.top_speed;
    speedProfile.response_ms = measured.response_ms;
    printf("\ndocs/tuning.md section 12, the %s simulator: measureTopSpeed says %.1f in/s, %.0f ms\n", robot,
           measured.top_speed, measured.response_ms);
    printf("(ends: + = stopped short of the target, - = went past it)\n");
    for (double inches : {6.0, 24.0, 48.0, 72.0}) {
        profileRow(Picture::fmt("PID_forward(%g), no profile", inches).c_str(), inches, 0);
        profileRow(Picture::fmt("PID_forward(%g), acceleration 150", inches).c_str(), inches, 150);
    }
    printf("DRIVE_RESPONSE_MS not what measureTopSpeed says, acceleration 150:\n");
    for (double factor : {0.7, 1.3}) {
        speedProfile.response_ms = measured.response_ms * factor;
        for (double inches : {24.0, 48.0})
            profileRow(Picture::fmt("PID_forward(%g), %.0f ms", inches, speedProfile.response_ms).c_str(), inches, 150);
    }
    speedProfile.top_speed = DRIVE_TOP_SPEED;
    speedProfile.response_ms = DRIVE_RESPONSE_MS;
}

// The picture: the robot's speed during PID_forward(48), without and with the profile (and the plan)
static void speedProfilePicture(const std::string &folder) {
    TopSpeedResult measured = measureTopSpeed();
    speedProfile.top_speed = measured.top_speed;
    speedProfile.response_ms = measured.response_ms;
    SpeedTrace pid_only = driveAndWatch(48, 0);
    speedProfile.acceleration = 150;
    SpeedTrace profiled = driveAndWatch(48, 0);
    speedProfile = {DRIVE_ACCELERATION, DRIVE_TOP_SPEED, DRIVE_RESPONSE_MS};

    // x: time in hundredths of a second (one per sample), y: inches/s; 2 s and 80 in/s
    Picture p{0, 0, 200, 80, 2.8};
    p.left = 50; p.bottom = 46; p.top = 56;
    for (double v = 0; v <= 80; v += 20) {
        p.line(0, v, 200, v, v == 0 ? AXIS : GRID, 1);
        p.text(0, v, Picture::fmt("%g", v), "end", INK_2, -6, 4, 11);
    }
    for (double s = 0; s <= 2; s += 0.5) {
        p.line(s * 100, 0, s * 100, -1.5, AXIS, 1);
        p.text(s * 100, 0, Picture::fmt("%g s", s), "middle", INK_2, 0, 18, 11);
    }
    p.text(0, 80, "speed, inches/s", "start", INK_2, -44, -10, 11);
    // The plan: up at 150 to the top speed, then down at 150 so it stops at 48
    double a = 150, top = measured.top_speed, up = top / a, cruise = (48 - top * up) / top;
    std::vector<std::pair<double, double>> plan = {{0, 0}, {up * 100, top}, {(up + cruise) * 100, top}, {(2 * up + cruise) * 100, 0}};
    std::string d;
    for (auto &pt : plan) d += Picture::fmt("%s%.1f %.1f ", d.empty() ? "M" : "L", p.X(pt.first), p.Y(pt.second));
    p.add(Picture::fmt("<path d='%s' fill='none' stroke='%s' stroke-width='1.5' stroke-dasharray='5 4'/>", d.c_str(), INK_2));
    auto trace = [&](const SpeedTrace &t, const char *color) {
        std::vector<std::pair<double, double>> points;
        for (size_t i = 0; i < t.speeds.size(); i++) points.push_back({(double)i, t.speeds[i]});
        p.path(points, color);
    };
    trace(pid_only, BLUE);
    trace(profiled, ORANGE);
    p.text(0, 80, Picture::fmt("<tspan fill='%s' font-weight='600'>no profile</tspan>: %s", BLUE,
                               controllerLine(pid_only.result).c_str()), "start", INK, 0, -32, 12);
    p.text(100, 80, Picture::fmt("<tspan fill='%s' font-weight='600'>acceleration 150</tspan>: %s", ORANGE,
                                 controllerLine(profiled.result).c_str()), "start", INK, 0, -32, 12);
    p.text((up + cruise / 2) * 100, top, "the plan (dashed)", "middle", INK_2, 0, -8, 11);
    p.save(folder, "speed-profile.svg", "The robot's speed during PID_forward(48): without a speed profile, and with acceleration 150");
}

static void speedProfileGuide(const std::string &folder) {
    speedProfilePicture(folder);
    speedProfileNumbers("perfect");
    // The realistic simulator (late and noisy readings, a drained battery, wheels that slip), like
    // tests/run_tests.sh's second runs: the profile was made for what it adds
    sim::realistic(true);
    sim::forgetReadings();
    speedProfileNumbers("realistic");
    sim::realistic(false);
    sim::forgetReadings();
}

// ---------- The arm and intake pictures and numbers in docs/mechanisms.md ----------

// A heavy arm: gravity pulls it down with this much of the motor's power (sim::mech_load), all the
// way up (like a lift; a real arm's pull changes with its angle)
const double ARM_LOAD = 15;

// The gains mechanisms.md shows. An Arm keeps the gains it was made with, so each has an Arm of its
// own, on a port of its own (the 36:1 cartridge an arm usually has).
struct ArmCase {
    const char *label;
    double kp, ki, kd;
    int port;
};
static ArmCase arm_cases[] = {
    {"the config's gains", ARM_KP, ARM_KI, ARM_KD, PORT12},
    {"kP 15", 15, ARM_KI, ARM_KD, PORT13},
    {"kD 0", ARM_KP, ARM_KI, 0, PORT14},
    {"kD 20", ARM_KP, ARM_KI, 20, PORT18},
    {"kD 40", ARM_KP, ARM_KI, 40, PORT15},
    {"kI 0", ARM_KP, 0, ARM_KD, PORT16},
    {"kP 0.5", 0.5, ARM_KI, ARM_KD, PORT17},
};
const int ARM_CASES = sizeof(arm_cases) / sizeof(arm_cases[0]);
#define CASE_MOTOR(i) motor(arm_cases[i].port, ratio36_1, false)
static motor case_motors[ARM_CASES] = {CASE_MOTOR(0), CASE_MOTOR(1), CASE_MOTOR(2), CASE_MOTOR(3),
                                       CASE_MOTOR(4), CASE_MOTOR(5), CASE_MOTOR(6)};
#define CASE_ARM(i) Arm(case_motors[i], arm_cases[i].kp, arm_cases[i].ki, arm_cases[i].kd)
static Arm case_arms[ARM_CASES] = {CASE_ARM(0), CASE_ARM(1), CASE_ARM(2), CASE_ARM(3), CASE_ARM(4), CASE_ARM(5), CASE_ARM(6)};

// One moveTo: where the arm really was every 10 ms (the motor's degrees), until 0.8 s after it finished
// (past the 0.5 s the motor takes the last bit, ARM_FINISH_MS in src/mechanisms.cpp, to where it holds)
struct ArmTrace {
    std::vector<double> positions;
    double time_ms = 0;    // until isDone
    double overshoot = 0;  // degrees past the target
    double error = 0;      // target - where it ended, 0.8 s later (+ = short of it)
    bool arrived = false;  // false: gave up after ARM_TIMEOUT_MS
};
static ArmTrace moveArm(int c, double from, double to, int start_ms, double max_speed = 100) {
    Arm &arm = case_arms[c];
    int port = arm_cases[c].port;
    sim::mech_load[port] = ARM_LOAD;
    // Put it exactly at from, held, so every run starts from where the table says (driving there
    // with the case's gains could give up on the way). The simulated arm is where its motor says it
    // is, so a new zero moves it there; the moveTo to where it now is only makes the Arm hold it.
    arm.resetPosition(from);
    arm.moveTo(from);
    arm.waitUntilDone();
    vexDelay(300 + start_ms);
    ArmTrace t;
    double start = sim::t_ms, direction = to > from ? 1 : -1;
    arm.moveTo(to, max_speed);
    if (arm.isDone()) {
        // More Arms than MAX_MECHANISMS (the example's arm counts too): this one does nothing, and
        // its numbers would look perfect
        printf("Arm case '%s' isn't running: too many Arms?\n", arm_cases[c].label);
        _exit(1);
    }
    while (!arm.isDone()) {
        t.positions.push_back(sim::mech_pos[port]);
        t.overshoot = fmax(t.overshoot, direction * (sim::mech_pos[port] - to));
        vexDelay(10);
    }
    t.time_ms = sim::t_ms - start;
    t.arrived = arm.waitUntilDone();
    for (int i = 0; i < 80; i++) {
        t.positions.push_back(sim::mech_pos[port]);
        vexDelay(10);
    }
    t.error = direction * (to - sim::mech_pos[port]);
    return t;
}

static std::string armResult(const ArmTrace &t) {
    if (!t.arrived) return Picture::fmt("gave up after %.1f s, %.0f° short", t.time_ms / 1000, t.error);
    return Picture::fmt("%.2f s, %.1f° past, ends %.1f° %s", t.time_ms / 1000, t.overshoot, fabs(t.error),
                        t.error >= 0 ? "short" : "past");
}

// One moveTo with the config's gains (gray) and with case c's: degrees over time, near the target
static void armPicture(const std::string &folder, const char *file, const char *title, int c, double from, double to) {
    ArmTrace good = moveArm(0, from, to, 0), bad = moveArm(c, from, to, 0);
    double low = to - 60, high = to + 30; // the last 60 degrees on the way, and 30 past
    if (to < from) { low = to - 30; high = to + 60; }
    // x: hundredths of a second (one per sample), 2.5 s; y: degrees
    Picture p{0, low, 250, high, 2.4};
    p.left = 56; p.top = 64; p.bottom = 46; p.right = 24;
    for (double d = ceil(low / 10) * 10; d <= high; d += 10) {
        p.line(0, d, 250, d, GRID, 1);
        p.text(0, d, Picture::fmt("%g°", d), "end", INK_2, -6, 4, 11);
    }
    for (double s = 0; s <= 2.5; s += 0.5) {
        p.line(s * 100, low, s * 100, low - 1.5, AXIS, 1);
        p.text(s * 100, low, Picture::fmt("%g s", s), "middle", INK_2, 0, 18, 11);
    }
    // The target, and ARM_TOLERANCE around it: inside it for ARM_SETTLE_MS counts as there
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='%s' fill-opacity='0.12'/>", p.X(0),
                       p.Y(to + ARM_TOLERANCE), 250 * p.scale, 2 * ARM_TOLERANCE * p.scale, AQUA));
    p.line(0, to, 250, to, AQUA, 1.2, " stroke-dasharray='5 4'");
    p.text(250, to, Picture::fmt("target %g°", to), "end", AQUA, 0, -6, 11);
    p.line(ARM_TIMEOUT_MS / 10, low, ARM_TIMEOUT_MS / 10, high, AXIS, 1, " stroke-dasharray='2 4'");
    p.text(ARM_TIMEOUT_MS / 10, high, "ARM_TIMEOUT_MS", "end", INK_2, -4, 12, 11);
    // The arm's position, cut off at the picture's edges
    p.add(Picture::fmt("<clipPath id='inside'><rect x='%.1f' y='%.1f' width='%.1f' height='%.1f'/></clipPath><g clip-path='url(#inside)'>",
                       p.X(0), p.Y(high), 250 * p.scale, (high - low) * p.scale));
    auto trace = [&](const ArmTrace &t, const char *color) {
        std::vector<std::pair<double, double>> points;
        for (size_t i = 0; i < t.positions.size(); i++) points.push_back({(double)i, t.positions[i]});
        p.path(points, color);
    };
    trace(good, AXIS);
    trace(bad, ORANGE);
    p.add("</g>");
    p.text(0, high, Picture::fmt("<tspan fill='%s' font-weight='600'>%s</tspan>: %s", ORANGE, arm_cases[c].label,
                                 armResult(bad).c_str()), "start", INK, -50, -38, 12);
    p.text(0, high, Picture::fmt("<tspan fill='%s' font-weight='600'>%s</tspan>: %s", INK_2, arm_cases[0].label,
                                 armResult(good).c_str()), "start", INK, -50, -20, 12);
    p.save(folder, file, title);
}

// The config's gains, all the way: up from 0 to 450, then down to 100
static void armGoodPicture(const std::string &folder) {
    ArmTrace up = moveArm(0, 0, 450, 0), down = moveArm(0, 450, 100, 0);
    // x: hundredths of a second, y: degrees / 4 (so 0..480 degrees fits next to 3.5 s)
    const double Q = 4;
    double seconds = (up.positions.size() + down.positions.size()) / 100.0;
    Picture p{0, 0, seconds * 100, 480 / Q, 2.2};
    p.left = 56; p.top = 64; p.bottom = 46; p.right = 24;
    for (double d = 0; d <= 480; d += 60) {
        p.line(0, d / Q, seconds * 100, d / Q, GRID, 1);
        p.text(0, d / Q, Picture::fmt("%g°", d), "end", INK_2, -6, 4, 11);
    }
    for (double s = 0; s <= seconds; s += 0.5) {
        p.line(s * 100, 0, s * 100, -1.5, AXIS, 1);
        p.text(s * 100, 0, Picture::fmt("%g s", s), "middle", INK_2, 0, 18, 11);
    }
    double switch_at = up.positions.size();
    for (auto target : {std::make_pair(0.0, 450.0), std::make_pair(switch_at, 100.0)})
        p.line(target.first, target.second / Q, target.first + (target.first ? seconds * 100 - switch_at : switch_at),
               target.second / Q, AQUA, 1.2, " stroke-dasharray='5 4'");
    std::vector<std::pair<double, double>> points;
    for (size_t i = 0; i < up.positions.size(); i++) points.push_back({(double)i, up.positions[i] / Q});
    for (size_t i = 0; i < down.positions.size(); i++) points.push_back({switch_at + i, down.positions[i] / Q});
    p.path(points, ORANGE);
    p.text(0, 480 / Q, Picture::fmt("<tspan fill='%s' font-weight='600'>moveTo(450)</tspan>: %s", ORANGE, armResult(up).c_str()),
           "start", INK, -50, -38, 12);
    p.text(0, 480 / Q, Picture::fmt("<tspan fill='%s' font-weight='600'>then moveTo(100)</tspan>: %s", ORANGE,
                                    armResult(down).c_str()), "start", INK, -50, -20, 12);
    p.save(folder, "arm-good.svg", "A heavy arm with the config's gains: moveTo(450), then moveTo(100)");
}

// One line of the table: case c's move from -> to, over STARTS runs
static void armRow(const std::string &label, int c, double from, double to, double max_speed = 100) {
    Range time, overshoot, error;
    int gave_up = 0;
    for (int start = 0; start < STARTS; start++) {
        ArmTrace t = moveArm(c, from, to, start, max_speed);
        time.add(t.time_ms / 1000);
        overshoot.add(t.overshoot);
        error.add(t.error);
        if (!t.arrived) gave_up++;
    }
    printf("  %-20s %-4s %s s, %s° past, ends %s°%s\n", label.c_str(), to > from ? "up" : "down",
           time.show("%.2f").c_str(), overshoot.show().c_str(), error.show().c_str(),
           gave_up ? Picture::fmt(", gave up %d of %d", gave_up, STARTS).c_str() : "");
}

// The table: each case's moves up and down
static void armNumbers(const char *robot) {
    printf("\ndocs/mechanisms.md, the %s simulator: an arm with %.0f%% gravity, moveTo from 0 to 450 (up) and 450 to 100 (down)\n",
           robot, ARM_LOAD);
    printf("(ends: + = short of the target, - = past it; gave up = after ARM_TIMEOUT_MS)\n");
    for (int c = 0; c < ARM_CASES; c++) {
        armRow(arm_cases[c].label, c, 0, 450);
        armRow(arm_cases[c].label, c, 450, 100);
    }
    // Going down slower: gravity carries it past less
    for (double speed : {50.0, 30.0}) {
        armRow(Picture::fmt("config, max_speed %g", speed), 0, 450, 100, speed);
    }
}

// The intake: what the motor draws and how fast it turns while it starts up, pulls in a heavy piece,
// and gets stuck. The jam check (src/mechanisms.cpp) looks at exactly these two readings.
static void intakePicture(const std::string &folder) {
    int port = intakeMotor.port;
    struct Sample { double amps, speed; bool backing_off; };
    std::vector<Sample> samples;
    intake.spin(100);
    for (int i = 0; i < 300; i++) {
        double t = i * 10;
        sim::mech_load[port] = (t >= 500 && t < 1000) ? 80 : 0; // a heavy piece, 0.5 to 1 s
        sim::mech_jammed[port] = t >= 1300 && t < 2000;        // stuck, 1.3 to 2 s
        samples.push_back({intakeMotor.current(currentUnits::amp), intakeMotor.velocity(velocityUnits::pct),
                           sim::cmd_in.mech_cmd[port] < 0});
        vexDelay(10);
    }
    intake.stop();
    sim::mech_load[port] = 0;
    sim::mech_jammed[port] = false;

    // Two graphs above each other: current (amps) on top, speed (percent) below. x: hundredths of a second.
    // y: the speed graph is -100..100 percent at 0..100, the current graph 0..2.5 A at 130..230.
    auto speedY = [](double pct) { return 50 + pct / 2; };
    auto ampsY = [](double amps) { return 130 + amps * 40; };
    Picture p{0, 0, 300, 230, 2.2};
    p.left = 62; p.top = 40; p.bottom = 46; p.right = 24;
    // When it backed off (shaded), in both graphs
    for (size_t i = 0; i < samples.size(); i++)
        if (samples[i].backing_off)
            p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='%s' fill-opacity='0.12'/>", p.X(i),
                               p.Y(230), p.scale, 230 * p.scale, ORANGE));
    for (double a = 0; a <= 2.5; a += 0.5) {
        p.line(0, ampsY(a), 300, ampsY(a), GRID, 1);
        p.text(0, ampsY(a), Picture::fmt("%.1f A", a), "end", INK_2, -6, 4, 11);
    }
    for (double s = -100; s <= 100; s += 50) {
        p.line(0, speedY(s), 300, speedY(s), s == 0 ? AXIS : GRID, 1);
        p.text(0, speedY(s), Picture::fmt("%g%%", s), "end", INK_2, -6, 4, 11);
    }
    for (double s = 0; s <= 3; s += 0.5) {
        p.line(s * 100, 0, s * 100, -1.5, AXIS, 1);
        p.text(s * 100, 0, Picture::fmt("%g s", s), "middle", INK_2, 0, 18, 11);
    }
    // The jam limits: more current than this, and slower than this (either way)
    p.line(0, ampsY(INTAKE_JAM_CURRENT), 300, ampsY(INTAKE_JAM_CURRENT), AQUA, 1.2, " stroke-dasharray='5 4'");
    p.text(300, ampsY(INTAKE_JAM_CURRENT), "INTAKE_JAM_CURRENT", "end", AQUA, 0, -5, 11);
    for (double s : {INTAKE_JAM_SPEED, -INTAKE_JAM_SPEED})
        p.line(0, speedY(s), 300, speedY(s), AQUA, 1.2, " stroke-dasharray='5 4'");
    p.text(300, speedY(INTAKE_JAM_SPEED), "INTAKE_JAM_SPEED", "end", AQUA, 0, -5, 11);
    std::vector<std::pair<double, double>> amps, speed;
    for (size_t i = 0; i < samples.size(); i++) {
        amps.push_back({(double)i, ampsY(samples[i].amps)});
        speed.push_back({(double)i, speedY(samples[i].speed)});
    }
    p.path(amps, BLUE);
    p.path(speed, BLUE);
    p.text(0, 230, "current, per motor", "start", INK_2, -56, -22, 11);
    p.text(0, 100, "speed", "start", INK_2, -56, -8, 11);
    // What happens when, between the two graphs (the shading: backing off to free it)
    p.text(25, 115, "starting up", "middle", INK, 0, 4, 12);
    p.text(75, 115, "a heavy piece", "middle", INK, 0, 4, 12);
    p.text(165, 115, "stuck: backs off, tries again", "middle", INK, 0, 4, 12);
    p.text(255, 115, "free again", "middle", INK, 0, 4, 12);
    p.save(folder, "intake-jam.svg", "An intake starting up, pulling in a heavy piece, and getting stuck: its current and speed");
}

// The intake's numbers: what a stuck motor draws at a few powers, and when a jam is noticed
static void intakeNumbers() {
    int port = intakeMotor.port;
    printf("\ndocs/mechanisms.md: a stuck intake motor in the simulator\n");
    for (double power : {100.0, 75.0, 50.0, 25.0}) {
        intake.spin(power);
        vexDelay(500);
        sim::mech_jammed[port] = true;
        double stuck_at = sim::t_ms, noticed = -1;
        int jams = intake.jamCount();
        while (sim::t_ms - stuck_at < 1000) {
            if (noticed < 0 && intake.jamCount() > jams) noticed = sim::t_ms - stuck_at;
            vexDelay(10);
        }
        double amps = intakeMotor.current(currentUnits::amp);
        if (sim::cmd_in.mech_cmd[port] < 0) { vexDelay(INTAKE_UNJAM_MS); amps = intakeMotor.current(currentUnits::amp); }
        printf("  spin(%3.0f): stuck, draws %.2f A: %s\n", power, amps,
               noticed < 0 ? "not noticed" : Picture::fmt("noticed after %.0f ms", noticed).c_str());
        sim::mech_jammed[port] = false;
        intake.stop();
        vexDelay(300);
    }
}

static void mechanismsGuide(const std::string &folder) {
    // The realistic simulator: its late readings are what make an arm swing (the perfect one forgives
    // a lot, see the numbers)
    sim::realistic(true);
    sim::forgetReadings();
    armGoodPicture(folder);
    armPicture(folder, "arm-kp-too-big.svg", "An arm moving up to 450° with kP 15: swings around the target", 1, 0, 450);
    armPicture(folder, "arm-kd-zero.svg", "An arm moving down to 100° with kD 0: gravity carries it past", 2, 450, 100);
    armPicture(folder, "arm-ki-zero.svg", "A heavy arm moving up to 450° with kI 0: stops short", 5, 0, 450);
    armPicture(folder, "arm-integral-range.svg", "A heavy arm with kP 0.5: stops further short than ARM_INTEGRAL_RANGE", 6, 0, 450);
    armNumbers("realistic");
    sim::realistic(false);
    sim::forgetReadings();
    armNumbers("perfect");
    intakePicture(folder);
    intakeNumbers();
}

int main(int argc, char **argv) {
    std::string folder = argc > 1 ? argv[1] : "docs/images";
    calibrateInertial();
    vex::task record(recorder);
    fieldCoordinates(folder);
    robotOffsets(folder);
    wallReset(folder);
    boomerang(folder);
    poseLeads(folder);
    chainedSquare(folder);
    tuningGraphs(folder);
    numbers();
    tuningNumbers();
    autonomousExample(folder);
    speedProfileGuide(folder);
    mechanismsGuide(folder);
    fflush(stdout);
    _exit(0); // the library's background tasks never end
}
