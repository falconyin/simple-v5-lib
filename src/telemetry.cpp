#include "simpleV5lib.h"
#include <atomic>
#include <cstdio>

// Changed from your code, read by the background task that runs the movements
static std::atomic<bool> terminal_on(false);
static std::atomic<bool> screen_on(false);

// Terminal
const double PRINT_EVERY_MS = 20;
static double last_print_time = 0;

// Brain screen graph. The drawing area is 480 x 240 pixels, y = 0 is the top.
const int SCREEN_WIDTH = 480;
const int GRAPH_MIDDLE = 130;     // y of the 0 line
const int GRAPH_HALF_HEIGHT = 100; // pixels from the 0 line to the top / bottom of the graph
static double graph_error_scale = 1; // the error that is drawn at the top of the graph
static double graph_timeout = 1;
static int last_x = 0;
static int last_error_y = GRAPH_MIDDLE;
static int last_output_y = GRAPH_MIDDLE;

void logToTerminal(bool on) {
    terminal_on = on;
}

void graphOnScreen(bool on) {
    screen_on = on;
}

// value / scale = 1 is the top of the graph, -1 the bottom
static int toScreenY(double value, double scale) {
    double y = GRAPH_MIDDLE - value / scale * GRAPH_HALF_HEIGHT;
    return (int)fmax(GRAPH_MIDDLE - GRAPH_HALF_HEIGHT, fmin(GRAPH_MIDDLE + GRAPH_HALF_HEIGHT, y));
}

void telemetryStart(const char* name, double target, double start_error, double timeout_ms) {
    if (terminal_on) {
        printf("\n# %s target=%.2f\n", name, target);
        printf("time_ms,error,speed,output,p,i,d\n");
        fflush(stdout);
        last_print_time = -PRINT_EVERY_MS; // print the first line right away
    }
    if (screen_on) {
        graph_error_scale = fmax(fabs(start_error), 1);
        graph_timeout = timeout_ms;
        last_x = 0;
        last_error_y = toScreenY(start_error, graph_error_scale);
        last_output_y = GRAPH_MIDDLE;

        Brain.Screen.clearScreen();
        Brain.Screen.setPenColor(color::white);
        // (the "true" picks the printAt that takes extra values; without it the call is ambiguous)
        Brain.Screen.printAt(5, 20, true, "%s  target %.1f", name, target);
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.printAt(300, 20, "error");
        Brain.Screen.setPenColor(color::green);
        Brain.Screen.printAt(380, 20, "power");
        Brain.Screen.setPenColor(color::white);
        Brain.Screen.drawLine(0, GRAPH_MIDDLE, SCREEN_WIDTH, GRAPH_MIDDLE); // the 0 line
    }
}

void telemetryUpdate(double time_ms, double error, double speed, double output, const PIDController &pid) {
    if (terminal_on && time_ms - last_print_time >= PRINT_EVERY_MS) {
        printf("%.0f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n", time_ms, error, speed, output, pid.last_p, pid.last_i, pid.last_d);
        fflush(stdout);
        last_print_time = time_ms;
    }
    if (screen_on) {
        // The whole timeout fits across the screen
        int x = (int)(time_ms / graph_timeout * SCREEN_WIDTH);
        int error_y = toScreenY(error, graph_error_scale);
        int output_y = toScreenY(output, 100);
        Brain.Screen.setPenColor(color::red);
        Brain.Screen.drawLine(last_x, last_error_y, x, error_y);
        Brain.Screen.setPenColor(color::green);
        Brain.Screen.drawLine(last_x, last_output_y, x, output_y);
        last_x = x;
        last_error_y = error_y;
        last_output_y = output_y;
    }
}
