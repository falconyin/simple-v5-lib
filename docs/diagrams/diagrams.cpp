// Draws the diagrams in docs/images as SVG files. The path diagrams come from the simulator
// (tests/sim/vex.h): the library's own code drives those movements, and the picture shows where the
// simulated robot went. The others (coordinates, offsets, the wall) are drawn from fixed numbers.
// Run docs/diagrams/make_diagrams.sh after changing how the robot drives, and look at the result.

#include "vex.h"
#include "simpleV5lib.h"
#include <string>
#include <vector>

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

// Stop, and put the robot at (0, 0) facing 0 (like the tests' reset)
static void startAtOrigin() {
    stopDriving();
    vexDelay(500);
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
    // Front distance sensor, 6 inches ahead and 2 to the right
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' rx='2' fill='%s'/>",
                       p.X(2 - 0.8), p.Y(6 + 0.5), 1.6 * p.scale, 1.0 * p.scale, AQUA));
    p.line(0, 6, 2, 6, AQUA, 1.5, " stroke-dasharray='3 3'");
    p.line(-1, 0, -1, 6, AQUA, 1.5, " stroke-dasharray='3 3'");
    p.arrow(2, 6.5, 2, 11.5, AQUA, 1.5);

    // The labels, to the right of the robot
    double lx = 11;
    p.text(lx, 9, "<tspan font-weight='600'>Front distance sensor</tspan>");
    p.text(lx, 9, "DISTANCE_FRONT_AHEAD = 6   (ahead: +)", "start", INK_2, 0, 18);
    p.text(lx, 9, "DISTANCE_FRONT_RIGHT = 2   (right: +)", "start", INK_2, 0, 36);
    p.line(2.9, 6, lx - 0.5, 8.6, AQUA, 1);
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
    double wall = 72, x = 50, y = 10, heading = 80, ahead = 6, right = 2;
    p.add(Picture::fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='%s'/>",
                       p.X(wall), p.Y(26), 6 * p.scale, 30 * p.scale, GRID));
    p.line(wall, -4, wall, 26, INK, 3);
    p.text(wall, 24, "wall at x = 72", "start", INK, 8, 4);

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
    p.text(wall, sensor.second, "10° off straight (at most 20)", "end", ORANGE, -8, 18, 12);

    p.text(30, -4, "x = 72 - reading × sin(80°) - sensor offset in x", "start", INK, 0, 30, 13);
    p.bottom = 50;
    p.save(folder, "wall-reset.svg", "resetXFromWall: the wall's x, back along the beam, then from the sensor to the robot's center");
}

// The carrot for a robot at (x, y) driving forward to the pose (tx, ty, th): the same rule as aimPoint
// in src/simpleV5lib.cpp (which is static, so it can't be called from here). Returns whether the
// look-ahead moved it.
static bool carrot(double x, double y, double tx, double ty, double th, double lead, double &cx, double &cy) {
    double line_x = sin(th * M_PI / 180), line_y = cos(th * M_PI / 180);
    double behind = lead * hypot(tx - x, ty - y);
    double back = (tx - x) * line_x + (ty - y) * line_y, beside = (tx - x) * line_y - (ty - y) * line_x;
    double look_ahead = 2 * POINT_AIM_DISTANCE;
    bool moved = false;
    if (fabs(beside) < look_ahead) {
        double limit = fmax(back - sqrt(look_ahead * look_ahead - beside * beside), 0);
        moved = limit < behind;
        behind = fmin(behind, limit);
    }
    cx = tx - behind * line_x;
    cy = ty - behind * line_y;
    return moved;
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
    p.text(9.5, 0.8, "near the line: the carrot stays 12 in ahead", "start", INK, 0, 4, 12);
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
    fflush(stdout);
    _exit(0); // the library's background tasks never end
}
