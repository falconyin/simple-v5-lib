#include "simpleV5lib.h"
#include <atomic>
#include <cstdio>
#include <cstring>

// Changed from your code, read by the background task that runs the movements
static std::atomic<bool> terminal_on(false);
static std::atomic<bool> screen_on(false);
static std::atomic<bool> sd_card_on(false);

// Terminal
const double PRINT_EVERY_MS = 20;
static double last_print_time = 0;

// SD card. Writing to it is slow, so the lines wait in sd_buffer and telemetryWriteSDCard()
// writes them when no movement is running. Only the background task touches the buffer.
static char sd_file[16] = "";     // the file of this program run, set once by logToSDCard
const int SD_BUFFER_SIZE = 32768; // about 10 seconds of movements
static char sd_buffer[SD_BUFFER_SIZE];
static int sd_buffer_used = 0;
static bool sd_this_movement = false; // the movement that is running now is being logged
static int sd_movement_number = 0;
static const char* sd_movement_name = "";
static double sd_target = 0;
static double last_sd_time = 0;

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

bool logToSDCard(bool on) {
    if (!on) {
        sd_card_on = false; // what is still in the buffer gets written anyway
        return true;
    }
    if (!Brain.SDcard.isInserted()) {
        printf("logToSDCard: no SD card in the Brain\n");
        fflush(stdout);
        sd_card_on = false;
        return false;
    }
    if (sd_file[0] == 0) {
        // A new file for this program run: the first number that isn't taken yet
        const int MOST_FILES = 9999;
        char name[sizeof(sd_file)];
        int number = 1;
        for (; number <= MOST_FILES; number++) {
            snprintf(name, sizeof(name), "pidlog%d.csv", number);
            if (!Brain.SDcard.exists(name)) {
                break;
            }
        }
        if (number > MOST_FILES) {
            printf("logToSDCard: the SD card already has pidlog1.csv to pidlog%d.csv, delete some\n", MOST_FILES);
            fflush(stdout);
            return false;
        }
        char header[] = "move,name,time_ms,target,error,speed,output,p,i,d,x,y,heading\n";
        int length = strlen(header);
        if (Brain.SDcard.savefile(name, (uint8_t*)header, length) != length) {
            printf("logToSDCard: could not make %s on the SD card\n", name);
            fflush(stdout);
            return false;
        }
        strcpy(sd_file, name);
    }
    printf("Logging every movement to %s on the SD card\n", sd_file);
    fflush(stdout);
    sd_card_on = true;
    return true;
}

void telemetryWriteSDCard() {
    if (sd_buffer_used == 0) {
        return;
    }
    if (Brain.SDcard.appendfile(sd_file, (uint8_t*)sd_buffer, sd_buffer_used) != sd_buffer_used) {
        printf("logToSDCard: could not write to %s, was the SD card taken out? Logging stopped.\n", sd_file);
        fflush(stdout);
        sd_card_on = false;
        sd_this_movement = false; // also for the rest of this movement: don't keep trying while it drives
    }
    sd_buffer_used = 0;
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
    sd_this_movement = sd_card_on;
    if (sd_this_movement) {
        sd_movement_number++;
        sd_movement_name = name;
        sd_target = target;
        last_sd_time = -PRINT_EVERY_MS;
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
    if (sd_this_movement && time_ms - last_sd_time >= PRINT_EVERY_MS) {
        char line[256];
        int length = snprintf(line, sizeof(line), "%d,%s,%.0f,%.2f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                              sd_movement_number, sd_movement_name, time_ms, sd_target, error, speed, output,
                              pid.last_p, pid.last_i, pid.last_d, getX(), getY(), getInertial());
        if (length > 0 && length < (int)sizeof(line)) { // (a line with absurdly huge numbers is left out)
            if (sd_buffer_used + length > SD_BUFFER_SIZE) {
                telemetryWriteSDCard(); // a very long movement: the buffer is full, it can't wait
            }
            if (sd_this_movement) { // (false if that write failed)
                memcpy(sd_buffer + sd_buffer_used, line, length);
                sd_buffer_used += length;
            }
        }
        last_sd_time = time_ms;
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
