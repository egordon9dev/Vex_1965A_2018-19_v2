#include "main.h"
#include "setup.hpp"
using pros::delay;
pros::Mutex DLMtx, DRMtx, DSMtx;
// these are shared between tasks
double DLEncBias = 0.0, DREncBias = 0.0, DSEncBias = 0.0;
pros::Motor mtr1(20);  // DR top
pros::Motor mtr2(8);   // DR bottom
pros::Motor mtr4(10);  // DL top
pros::Motor mtr5(9);   // DL bottom
double getDL() {
    DLMtx.take(50);
    double d = (mtr4.get_position()) + DLEncBias;
    DLMtx.give();
    return d;
}
double getDR() {
    DRMtx.take(50);
    double d = (-mtr1.get_position()) + DREncBias;
    DRMtx.give();
    return d;
}
double getDS() {
    DSMtx.take(50);
    double d = (perpindicularWheelEnc->get_value()) + DSEncBias;
    DSMtx.give();
    return d;
}
double getDLVel() {
    DLMtx.take(50);
    double d = mtr4.get_actual_velocity();
    DLMtx.give();
    return d;
}
double getDRVel() {
    DRMtx.take(50);
    double d = -mtr1.get_actual_velocity();
    DRMtx.give();
    return d;
}
double getDriveVel() { return 0.5 * (getDLVel() + getDRVel()); }
void zeroDriveEncs() {
    double curDL = getDL(), curDR = getDR(), curDS = getDS();
    DLMtx.take(50);
    DLEncBias -= curDL;
    DLMtx.give();
    DRMtx.take(50);
    DREncBias -= curDR;
    DRMtx.give();
    DSMtx.take(50);
    DSEncBias -= curDS;
    DSMtx.give();
}
// requested voltages are shared between tasks
int DL_requested_voltage = 0, DR_requested_voltage = 0, driveLim = 12000;
void setDR(int n) {
    n = clamp(n, -driveLim, driveLim);
    n = DRSlew.update(n);
    n = drSaver.getPwr(n, getDR());
    DRMtx.take(50);
    mtr1.move_voltage(-n);
    mtr2.move_voltage(n);
    DR_requested_voltage = n;
    DRMtx.give();
}
void setDL(int n) {
    n = clamp(n, -driveLim, driveLim);
    n = DLSlew.update(n);
    n = dlSaver.getPwr(n, getDL());
    DLMtx.take(50);
    mtr4.move_voltage(n);
    mtr5.move_voltage(-n);
    DL_requested_voltage = n;
    DLMtx.give();
}
void testDriveMtrs() {
    while (true) {
        int pwr = 4000;
        setDL(0);
        setDR(0);
        DRMtx.take(50);
        mtr1.move_voltage(pwr);
        DRMtx.give();
        delay(400);

        setDL(0);
        setDR(0);
        DRMtx.take(50);
        mtr2.move_voltage(pwr);
        DRMtx.give();
        delay(400);
        setDL(0);
        setDR(0);
        DLMtx.take(50);
        mtr4.move_voltage(pwr);
        DLMtx.give();
        delay(400);

        setDL(0);
        setDR(0);
        DLMtx.take(50);
        mtr5.move_voltage(pwr);
        DLMtx.give();
        delay(400);
        setDL(0);
        setDR(0);
        delay(500);
        printf(".\n");
    }
}
int getDRVoltage() {
    DRMtx.take(50);
    int n = DR_requested_voltage;
    DRMtx.give();
    return n;
}
int getDLVoltage() {
    DLMtx.take(50);
    int n = DL_requested_voltage;
    DLMtx.give();
    return n;
}