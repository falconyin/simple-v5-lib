#include "simpleV5lib.h"

const int MAX_AUTONS = 10;

static const char* auton_names[MAX_AUTONS];
static void (*auton_routines[MAX_AUTONS])();
static int auton_count = 0;
static int selected = 0;
static bool selector_running = false;

void addAuton(const char* name, void (*routine)()) {
    if (auton_count >= MAX_AUTONS) {
        return;
    }
    auton_names[auton_count] = name;
    auton_routines[auton_count] = routine;
    auton_count++;
}

// Show the selected routine on the Brain screen and the controller screen
static void showSelection() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Auton %d/%d: %s", selected + 1, auton_count, auton_names[selected]);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("<  tap left half              tap right half  >");

    Controller.Screen.clearLine(1);
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("%s", auton_names[selected]);
}

// Runs in the background (as a task) until autonomous or driver control starts
static int selectorLoop() {
    showSelection();
    while (selector_running) {
        int step = 0; // -1 = previous routine, 1 = next routine
        if (Brain.Screen.pressing()) {
            step = (Brain.Screen.xPosition() < 240) ? -1 : 1; // the screen is 480 pixels wide
            while (Brain.Screen.pressing()) {
                vexDelay(10); // wait for the finger to lift, so one tap = one step
            }
        } else if (Controller.ButtonLeft.pressing()) {
            step = -1;
            while (Controller.ButtonLeft.pressing()) {
                vexDelay(10);
            }
        } else if (Controller.ButtonRight.pressing()) {
            step = 1;
            while (Controller.ButtonRight.pressing()) {
                vexDelay(10);
            }
        }

        if (step != 0 && selector_running) {
            // + auton_count keeps the number positive when going back from the first routine
            selected = (selected + step + auton_count) % auton_count;
            showSelection();
        }
        vexDelay(20);
    }
    return 0;
}

void startAutonSelector() {
    if (auton_count == 0 || selector_running) {
        return;
    }
    selector_running = true;
    // Start selectorLoop in the background. Made with "new" so the task object is never destroyed,
    // which keeps the task running after this function returns.
    new task(selectorLoop);
}

void stopAutonSelector() {
    selector_running = false;
}

void runSelectedAuton() {
    stopAutonSelector();
    if (auton_count == 0) {
        return;
    }
    auton_routines[selected]();
}
