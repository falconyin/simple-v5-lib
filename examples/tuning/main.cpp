/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  Tuning helper: see what the PID does during a movement    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

int main() {
    calibrateInertial(); // keep the robot still for about 2 seconds

    graphOnScreen(true); // red = error, green = power, the white line is 0
    logToTerminal(true); // numbers in the terminal (connect with a USB cable)

    PID_turn(90, 0.5, 0.2);
    wait(3, seconds); // time to look at the graph before the next movement clears it
    PID_forward(24, 0.3, 0.2);

    // How to read the graph:
    // - Red line crosses the white line: the robot went past the target -> raise kD or lower kP
    // - Red line flattens out above the white line: the robot stopped short -> raise kI (or kP)
    // - Green line jumps up and down quickly: the robot is shaking -> lower kD
    //
    // To make a chart from the terminal: copy the lines from "time_ms,..." down,
    // paste them into a spreadsheet (one value per column), and insert a line chart.
    // The p, i and d columns show how much each part of the PID is pushing.
}
