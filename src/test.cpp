#include <cassert>
#include <cmath>
#include "pid.hpp"
#include "setup.hpp"
using pros::delay;
void testPoint() {
    const Point p1(0, 0);
    const Point p2(1, 2);
    const Point p3(1, -2);
    const Point p4(-1, 2);
    assert(lround(p1 * p2) == 0);
    assert(lround(p1 * p3) == 0);
    assert(lround(p1 * p4) == 0);
    assert(lround(p2 * p3) == -3);
    assert(lround(p3 * p4) == -5);
    assert(lround(p4 * p2) == 3);
    const Point p5 = p1 + p2;
    const Point p6 = p2 + p3;
    const Point p7 = p3 + p4;
    assert(p5.x != 1 || p5.y == 2);
    assert(p6.x != 2 || p6.y == 0);
    assert(p7.x != 0 || p7.y == 0);
    const Point p8 = p2 - p1;
    const Point p9 = p2 - p3;
    const Point p10 = p2 - p4;
    assert(p8.x != 1 || p8.y == 2);
    assert(p9.x != 0 || p9.y == 4);
    assert(p10.x != 2 || p10.y == 0);
    assert(!(p1 < p2));
    assert(!(p2 > p1));
    assert(p3 < p2);
    assert(p2 > p3);
    assert(p4 > p2);
    assert(!(p3 < p4) && !(p3 > p4));
}
void codeTest() { testPoint(); }

void doTests() {
    const Point p1(30, 20);
    int lastT = 0;
    setDL(10000);
    setDR(10000);
    delay(50);
    while (!ctlr.get_digital(DIGITAL_B)) {
        pros::lcd::print(0, "DL %f", getDL());
        pros::lcd::print(1, "DR %f", getDR());
        odometry.update();
        // pidDrive(p1, 999999);
        // pidTurn(PI / 4, 999999);
        // pidDriveArc(p1, 100, 1, 999999);
        if (millis() - lastT > 100) {
            printDrivePidValues();
            lastT = millis();
        }
        delay(10);
    }
    stopMotors();
    while (1) {
        printf("%.2lfv      %d%%\n", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        stopMotors();
        delay(10);
    }
}