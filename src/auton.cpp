#include "auton.hpp"
#include "Point.hpp"
#include "main.h"
#include "setup.hpp"

using pros::delay;
using std::cout;
using std::endl;

void autonFlagSide(bool leftSide) {}

void autonomous() {
    odometry.reset();
    setDriveSlew(true);
    if (autoSel_nAuton == 0) {
        stopMotorsBlock();
    } else if (autoSel_nAuton == 1) {
        autonMainBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        autonSupportBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 3) {
        autonSupCrossBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 4) {
        autonFlagSide(autoSel_leftSide);
    }
    stopMotorsBlock();
}
