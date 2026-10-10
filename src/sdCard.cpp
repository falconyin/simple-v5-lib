#include "simpleV5lib.h"
#include <cstdio>
#include <cstring>

// Saving and loading the PID gains on the Brain's SD card (saveGainsToSDCard, loadGainsFromSDCard).
// The file has one line per gain, with the value and what simpleV5LibConfig.h said when it was saved:
//   TURN_KP = 3.5 config 3.2

static const char* GAINS_FILE = "pid_gains.txt";
const int GAINS_FILE_MAX_SIZE = 2048;

// Every gain: its name in the file (the same as in simpleV5LibConfig.h), where it lives while
// the program runs, and its value in simpleV5LibConfig.h
struct GainSetting {
    const char* name;
    double* value;
    double config;
};

static GainSetting gain_settings[] = {
    {"TURN_KP", &turnGains.kp, TURN_KP},
    {"TURN_KI", &turnGains.ki, TURN_KI},
    {"TURN_KD", &turnGains.kd, TURN_KD},
    {"FORWARD_KP", &forwardGains.kp, FORWARD_KP},
    {"FORWARD_KI", &forwardGains.ki, FORWARD_KI},
    {"FORWARD_KD", &forwardGains.kd, FORWARD_KD},
    {"SWING_KP", &swingGains.kp, SWING_KP},
    {"SWING_KI", &swingGains.ki, SWING_KI},
    {"SWING_KD", &swingGains.kd, SWING_KD},
    {"ARC_KP", &arcGains.kp, ARC_KP},
    {"ARC_KI", &arcGains.ki, ARC_KI},
    {"ARC_KD", &arcGains.kd, ARC_KD},
};
const int GAIN_COUNT = sizeof(gain_settings) / sizeof(gain_settings[0]);

static GainSetting* findGain(const char* name) {
    for (int i = 0; i < GAIN_COUNT; i++) {
        if (strcmp(gain_settings[i].name, name) == 0) {
            return &gain_settings[i];
        }
    }
    return nullptr;
}

// The file stores the numbers with 10 digits, so "the same" allows for the last one
static bool sameNumber(double a, double b) {
    return fabs(a - b) <= 1e-8 * fmax(fabs(a), fabs(b));
}

bool saveGainsToSDCard() {
    if (!Brain.SDcard.isInserted()) {
        printf("saveGainsToSDCard: no SD card in the Brain\n");
        return false;
    }
    char text[GAINS_FILE_MAX_SIZE];
    int length = snprintf(text, sizeof(text),
                          "# PID gains saved by simple-v5-lib, loaded by loadGainsFromSDCard()\n"
                          "# name = gain config (what simpleV5LibConfig.h said when it was saved)\n");
    for (int i = 0; i < GAIN_COUNT; i++) {
        length += snprintf(text + length, sizeof(text) - length, "%s = %.10g config %.10g\n",
                           gain_settings[i].name, *gain_settings[i].value, gain_settings[i].config);
    }
    if (Brain.SDcard.savefile(GAINS_FILE, (uint8_t*)text, length) != length) {
        printf("saveGainsToSDCard: could not write %s\n", GAINS_FILE);
        return false;
    }
    printf("Gains saved to %s on the SD card\n", GAINS_FILE);
    fflush(stdout);
    return true;
}

bool loadGainsFromSDCard() {
    if (!Brain.SDcard.isInserted()) {
        printf("loadGainsFromSDCard: no SD card, using the gains from simpleV5LibConfig.h\n");
        return false;
    }
    if (!Brain.SDcard.exists(GAINS_FILE)) {
        printf("loadGainsFromSDCard: no %s on the SD card yet, using the gains from simpleV5LibConfig.h\n", GAINS_FILE);
        return false;
    }
    if (Brain.SDcard.size(GAINS_FILE) > GAINS_FILE_MAX_SIZE) {
        // Reading only the start of it could miss half a line: don't use any of it
        printf("loadGainsFromSDCard: %s is too big (over %d bytes), using the gains from simpleV5LibConfig.h\n",
               GAINS_FILE, GAINS_FILE_MAX_SIZE);
        return false;
    }
    char text[GAINS_FILE_MAX_SIZE + 1];
    int length = Brain.SDcard.loadfile(GAINS_FILE, (uint8_t*)text, GAINS_FILE_MAX_SIZE);
    if (length <= 0) {
        printf("loadGainsFromSDCard: could not read %s\n", GAINS_FILE);
        return false;
    }
    text[length] = 0;

    printf("Gains from %s on the SD card:\n", GAINS_FILE);
    char* line = text;
    while (line != nullptr && *line != 0) {
        char* next_line = strchr(line, '\n');
        if (next_line != nullptr) {
            *next_line = 0; // end the line here, so sscanf only reads this line
            next_line++;
        }
        char name[32];
        double value, config;
        int found = sscanf(line, " %31[A-Za-z_] = %lf config %lf", name, &value, &config);
        GainSetting* gain = (found >= 2) ? findGain(name) : nullptr;
        if (gain == nullptr) {
            // a comment, an empty line or a name we don't know: skip it
        } else if (!(value >= 0 && value < 1e6)) { // (also false for "nan")
            printf("  %s = %g? That can't be right, keeping %g\n", name, value, *gain->value);
        } else if (found == 3 && !sameNumber(config, gain->config)) {
            // simpleV5LibConfig.h was changed after these gains were saved: it is the newer one
            printf("  %s = %g from simpleV5LibConfig.h (it changed since %g was saved)\n", name, gain->config, value);
            *gain->value = gain->config;
        } else {
            printf("  %s = %g\n", name, value);
            *gain->value = value;
        }
        line = next_line;
    }
    fflush(stdout);
    return true;
}
