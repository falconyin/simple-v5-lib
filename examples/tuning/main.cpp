/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Description:  Tuning helper: change the PID gains from the controller   */
/*                  and see what the PID does during each movement            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

int main() {
    calibrateInertial(); // keep the robot still for about 2 seconds
    loadGainsFromSDCard(); // carry on from the gains of the last tuning session (if any)

    graphOnScreen(true); // red = error, green = power, the white line is 0
    logToTerminal(true); // numbers in the terminal (connect with a USB cable)
    logToSDCard(true);   // and in pidlog1.csv, pidlog2.csv, ... on the SD card

    // Tune from the controller (the robot moves when you press A, give it room):
    //   X             pick what to tune: TURN, FORWARD, SWING or ARC
    //   Up / Down     pick kP, kI or kD
    //   Right / Left  make it 10% bigger / smaller (hold to keep changing)
    //   A             try it: turn 90 degrees, drive 24 inches, ... The next try goes back.
    //   B             done: saves the gains on the SD card (loadGainsFromSDCard() above loads
    //                 them next time), and shows them on the Brain screen and the terminal to
    //                 copy into simpleV5LibConfig.h. Without an SD card they are lost when the
    //                 program stops!
    // The controller shows how the last try went, for example "1.25s ov2.3 e0.12":
    //   1.25 seconds to finish, went 2.3 past the target at most, ended 0.12 from it.
    tuneWithController();

    // How to tune, in short (docs/how-it-works.md has more):
    // 1. Set kI and kD to 0. Raise kP until the robot gets to the target quickly but
    //    swings back and forth around it.
    // 2. Raise kD until it stops swinging: the overshoot ("ov") gets close to 0.
    // 3. Stops a little short ("e" stays big)? Raise kI a little.
    //
    // How to read the graph:
    // - Red line crosses the white line: the robot went past the target -> raise kD or lower kP
    // - Red line flattens out above the white line: the robot stopped short -> raise kI (or kP)
    // - Green line jumps up and down quickly: the robot is shaking -> lower kD
    //
    // To make a chart from the terminal: copy the lines from "time_ms,..." down,
    // paste them into a spreadsheet (one value per column), and insert a line chart.
    // From the SD card: open the newest pidlog file (the highest number), filter the "move" column to one movement
    // and make a line chart of error and output against time_ms.
    // The p, i and d columns show how much each part of the PID is pushing.
}
