#include "setup.hpp"
#include <cassert>
#include <string>
#include "MotorSaver.hpp"
#include "Point.hpp"
#include "main.h"
#include "pid.hpp"

// motors
pros::Motor mtr1(20);  // DR top
pros::Motor mtr2(8);   // DR bottom
pros::Motor mtr3(19);  // intake
pros::Motor mtr4(10);  // DL top
pros::Motor mtr5(9);   // DL bottom
pros::Motor mtr6(17);  // flywheel
pros::Motor mtr7(14);  // drfb
pros::Motor mtr8(12);  // claw
/* bad ports:
5,
15(claw),
18(flywheel: during practice),
13(claw: during practice),
12 (claw: first time turning on the robot in the morning)
*/
// bad ports: 11, 12, 14, 15, 1, 2, 3, 4, 5, 6, 7

// motor savers
MotorSaver dlSaver(35);
MotorSaver drSaver(35);
MotorSaver drfbSaver(25);
MotorSaver clawSaver(35);
MotorSaver intakeSaver(40);
MotorSaver flySaver(40);

pros::Controller ctlr(pros::E_CONTROLLER_MASTER);
pros::Controller ctlr2(pros::E_CONTROLLER_PARTNER);
using pros::delay;
// sensors
pros::ADIPotentiometer* drfbPot;
pros::ADILineSensor* ballSensL;
pros::ADILineSensor* ballSensR;
pros::ADILineSensor* lineSens1;
pros::ADIEncoder* perpindicularWheelEnc;

//----------- Constants ----------------
// pot
const int drfbPotMaxPos = 3300, drfbPotPos0 = 1058, drfbPotMinPos = 1055, drfbPotPos1 = 2260, drfbPotPos1Plus = 2530, drfbPotPos2 = 2805, drfbPotPos2Plus = 3150;
const int drfbPotMinClaw0 = 1390, drfbPotMaxClaw0 = 1720, drfbPotMinClaw1 = 2080, drfbPot18Max = 1449;
// ime
const int drfbMaxPos = 2390, drfbPos0 = -30, drfbMinPos = -50, drfbPos1 = 1190, drfbPos1Plus = 1493, drfbPos2 = 1793, drfbPos2Plus = 2280;
const int drfbMinClaw0 = 350, drfbMaxClaw0 = 640, drfbMinClaw1 = 1087, drfb18Max = 350;

const double dShotSpeed1 = 2.55, dShotSpeed2 = 2.7;

const int dblClickTime = 450, claw180 = 1350, clawPos0 = 0, /*590*/ clawPos1 = claw180 /*3800*/;
const double ticksPerInch = 52.746 /*very good*/, ticksPerInchADI = 35.2426, ticksPerRadian = 368.309;
const double PI = 3.14159265358979323846;
const int BIL = 1000000000, MIL = 1000000;

//----------- MISC ----------------
int clamp(int n, int a, int b) { return n < a ? a : (n > b ? b : n); }
double clamp(double n, double a, double b) { return n < a ? a : (n > b ? b : n); }
Point polarToRect(double mag, double angle) {
    Point p(mag * cos(angle), mag * sin(angle));
    return p;
}

//----------- Drive -----------
double getDL() { return (mtr4.get_position() - mtr5.get_position()) * 0.5; }
double getDR() { return (-mtr1.get_position() + mtr2.get_position()) * 0.5; }
double getDS() { return perpindicularWheelEnc->get_value(); }
int millis() { return pros::millis(); }
int DL_requested_voltage = 0, DR_requested_voltage = 0, driveLim = 12000;
void setDR(int n) {
    n = clamp(n, -driveLim, driveLim);
    n = DRSlew.update(n);
    n = drSaver.getPwr(n, getDR());
    mtr1.move_voltage(-n);
    mtr2.move_voltage(n);
    DR_requested_voltage = n;
}
void setDL(int n) {
    n = clamp(n, -driveLim, driveLim);
    n = DLSlew.update(n);
    n = dlSaver.getPwr(n, getDL());
    mtr4.move_voltage(n);
    mtr5.move_voltage(-n);
    DL_requested_voltage = n;
}
void testDriveMtrs() {
    while (true) {
        int pwr = 4000;
        setDL(0);
        setDR(0);
        mtr1.move_voltage(pwr);
        delay(400);

        setDL(0);
        setDR(0);
        mtr2.move_voltage(pwr);
        delay(400);

        setDL(0);
        setDR(0);
        mtr4.move_voltage(pwr);
        delay(400);

        setDL(0);
        setDR(0);
        mtr5.move_voltage(pwr);
        delay(400);

        setDL(0);
        setDR(0);
        delay(500);
        printf(".\n");
    }
}
int getDRVoltage() { return DR_requested_voltage; }
int getDLVoltage() { return DL_requested_voltage; }

//------------ Intake ---------------
namespace intake {
int altT0;
}
void setIntake(int n) {  // +: front, -: back
    if (mtr3.get_current_draw() > 1500) int n = 0;
    n = clamp(n, -12000, 12000);
    static int prevFly = getFlywheel();
    /*if (getFlywheel() - prevFly < 15 && n < 0) n = 0;*/  // fix this
    n = intakeSlew.update(n);
    n = intakeSaver.getPwr(n, mtr3.get_position());
    mtr3.move_voltage(n);
    prevFly = getFlywheel();
}
void setIntake(IntakeState is) {
    static IntakeState prev = IntakeState::NONE;
    if (is == IntakeState::NONE) {
        setIntake(0);
    } else if (is == IntakeState::FRONT) {
        setIntake(12000);
    } else if (is == IntakeState::BACK) {
        setIntake(-12000);
    } else if (is == IntakeState::ALTERNATE) {
        if (isBallIn()) {
            setIntake(0);
        } else {
            if (prev != IntakeState::ALTERNATE) intake::altT0 = millis();
            if (((millis() - intake::altT0) / 250) % 3 < 2) {
                setIntake(-12000);
            } else {
                setIntake(12000);
            }
        }
    }
    prev = is;
}
IntakeState getISLoad() {
    if (isBallIn()) {
        return IntakeState::NONE;
    } else {
        return IntakeState::ALTERNATE;
    }
}
int getBallSensL() { return ballSensL->get_value(); }
int getBallSensR() { return ballSensR->get_value(); }
bool isBallIn() { return getBallSensL() < 2000 || getBallSensR() < 2000; }
bool isLineDetected() { return lineSens1->get_value() < 500; }

//----------- DRFB functions ---------
int drfb_requested_voltage = 0;
int drfbPowerLimit = 3500;
int drfbFullRangePowerLimit = 12000;
void setDrfb(int n) {
    n = clamp(n, -drfbFullRangePowerLimit, drfbFullRangePowerLimit);
    if (getDrfb() < drfbMinPos + 150 && n < -drfbPowerLimit) n = -drfbPowerLimit;
    if (getDrfb() > drfbMaxPos - 150 && n > drfbPowerLimit) n = drfbPowerLimit;
    n = drfbSlew.update(n);
    n = drfbSaver.getPwr(n, getDrfb());
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
void setDrfbDumb(int n) {
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
int getDrfbPot() { return 4095 - drfbPot->get_value(); }
double drfbIMEBias = 0;
double getDrfb() { return drfbIMEBias + mtr7.get_position(); }
void printDrfbSensors() { pros::lcd::print(3, "ime %f pot %d", getDrfb(), getDrfbPot()); }

int getDrfbEncoder() { return mtr7.get_position(); }
int getDrfbVoltage() { return drfb_requested_voltage; }
int getDrfbCurrent() { return mtr7.get_current_draw(); }
int drfbPidBias = 0;
bool pidDrfb(double pos, int wait) {
    drfbPid.sensVal = getDrfb();
    int out = drfbPid.update();
    setDrfb(drfbPidBias + out);
    if (drfbPid.doneTime + wait < millis()) return true;
}
void pidDrfb() { pidDrfb(drfbPid.target, 9999999); }
//---------- Claw functions --------
namespace clawOpctl {
double bias = 0.0;
}
int claw_requested_voltage = 0;
int clawPowerLimit = 12000;
void setClaw(int n) {
    n = clamp(n, -clawPowerLimit, clawPowerLimit);
    if (mtr8.get_current_draw() > 2500) int n = 0;
    if (getDrfb() < drfbMinClaw0 || (getDrfb() > drfbMaxClaw0 && getDrfb() < drfbMinClaw1)) n = 0;
    int maxPwr = 1200;
    if (getClaw() < 80 && n < -maxPwr) n = -maxPwr;
    if (getClaw() > claw180 - 80 && n > maxPwr) n = maxPwr;
    n = clawSaver.getPwr(n, mtr8.get_position());
    n = clawSlew.update(n);
    mtr8.move_voltage(n);
    claw_requested_voltage = n;
}
void setClawDumb(int n) {
    mtr8.move_voltage(n);
    claw_requested_voltage = n;
}
void setClawPosition(double pos) { clawOpctl::bias += pos - getClaw(); };
double getClaw() { return mtr8.get_position() + clawOpctl::bias; }
int getClawVoltage() { return claw_requested_voltage; }
bool pidClaw(double a, int wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}
void pidClaw() { pidClaw(clawPid.target, 999999); }
//--------- Flywheel functions --------
int flywheel_requested_voltage = 0;
void setFlywheel(int n) {
    n = clamp(n, 0, 12000);
    // n = flywheelSlew.update(n);
    // n = flySaver.getPwr(n, getFlywheel());
    mtr6.move_voltage(-n);
    flywheel_requested_voltage = n;
}
double getFlywheel() { return -mtr6.get_position(); }
double getFlywheelFromMotor() { return -3.1 / 200.0 * mtr6.get_actual_velocity(); }
int getFlywheelVoltage() { return flywheel_requested_voltage; }

double FWSpeeds[][2] = {{0, 0}, {1.0, 4200}, {2.0, 7700}, {2.2, 8400}, {2.4, 9100}, {2.5, 9850}, {2.7, 10600}, {2.8, 10600}, {2.9, 11480}};
bool pidFlywheel(double speed) {
    static double prevSpeed = 0.0;
    static int prevT = 0, prevPosition = 0;
    static double output = 0.0;
    static bool crossedTarget = false;
    static int dir = 1;
    static int prevUpdateT = -9999;
    static double sumVel = 0.0;
    static int numVel = 0;
    if (fabs(speed - prevSpeed) > 0.1) {
        output = 0.0;
        crossedTarget = false;
        if (speed > prevSpeed) {
            dir = 1;
        } else {
            dir = -1;
        }
    }
    prevSpeed = speed;
    double dt = millis() - prevT;
    prevT = millis();
    int FWSpeedsLen = sizeof(FWSpeeds) / sizeof(double[2]);
    int bias = 0;
    double smallestDist = 99999999;
    for (int i = 0; i < FWSpeedsLen; i++) {
        double dist = fabs(speed - FWSpeeds[i][0]);
        if (dist < smallestDist) {
            bias = FWSpeeds[i][1];
            smallestDist = dist;
        }
    }
    if (dt < 1000 && dt > 0) {
        sumVel += getFlywheelFromMotor();
        numVel++;
        if (millis() - prevUpdateT > 50) {  // 95

            flywheelPid.sensVal = sumVel / numVel;  //(getFlywheel() - prevPosition) / (millis() - prevUpdateT);
            sumVel = 0.0;
            numVel = 0;
            prevUpdateT = millis();
            prevPosition = getFlywheel();
            flywheelPid.target = speed;
            if (!crossedTarget) {
                if (flywheelPid.sensVal > flywheelPid.target - 0.1 * dir) {
                    if (dir == 1) {
                        crossedTarget = true;
                    } else {
                        setFlywheel(0);
                    }
                } else if (flywheelPid.sensVal < flywheelPid.target - 0.1 * dir) {
                    if (dir == -1) {
                        crossedTarget = true;
                    } else {
                        setFlywheel(12000);
                    }
                }
            }
            if (crossedTarget) {
                output += flywheelPid.update();
                output = clamp(output, -6000.0, 6000.0);
                setFlywheel(bias + lround(output));
            }
        }
    }
    return flywheelPid.doneTime < millis();
}
bool pidFlywheel(double speed, int wait) {
    pidFlywheel(speed);
    return flywheelPid.doneTime + wait < millis();
}
bool pidFlywheel() { return pidFlywheel(flywheelPid.target); }
//--------------------- Misc -----------------
const int ctlrIdxLeft = 0, ctlrIdxUp = 1, ctlrIdxRight = 2, ctlrIdxDown = 3, ctlrIdxY = 4, ctlrIdxX = 5, ctlrIdxA = 6, ctlrIdxB = 7, ctlrIdxL1 = 8, ctlrIdxL2 = 9, ctlrIdxR1 = 10, ctlrIdxR2 = 11;
const std::string clickIdxNames[] = {"Left", "Up", "Right", "Down", "Y", "X", "A", "B", "L1", "L2", "R1", "R2"};
const pros::controller_digital_e_t clickIdxIds[] = {DIGITAL_LEFT, DIGITAL_UP, DIGITAL_RIGHT, DIGITAL_DOWN, DIGITAL_Y, DIGITAL_X, DIGITAL_A, DIGITAL_B, DIGITAL_L1, DIGITAL_L2, DIGITAL_R1, DIGITAL_R2};
bool** getAllClicks() {
    static bool prevClicks[] = {false, false, false, false, false, false, false, false, false, false, false, false};
    static int prevTimes[] = {-9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999};
    bool curClicks[12];
    static bool dblClicks[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
    bool** allClicks = new bool*[3];
    for (int i = 0; i < 3; i++) { allClicks[i] = new bool[12]; }
    for (int i = 0; i < 12; i++) {
        curClicks[i] = ctlr.get_digital(clickIdxIds[i]);
        if (!curClicks[i]) dblClicks[i] = false;
        if (curClicks[i] && !prevClicks[i]) {
            // double tap
            if (millis() - prevTimes[i] < dblClickTime) dblClicks[i] = true;
            prevTimes[i] = millis();
        }
        allClicks[0][i] = prevClicks[i];
        allClicks[1][i] = curClicks[i];
        allClicks[2][i] = dblClicks[i];
        prevClicks[i] = curClicks[i];
    }
    return allClicks;
}
void printAllClicks(int line, bool** allClicks) {
    std::string line1 = "prevClicks: ";
    for (int i = 0; i < 12; i++) { line1 += (allClicks[0][i] ? (clickIdxNames[i] + ", ") : ""); }
    line1 = line1.substr(0, line1.size() - 2);
    std::string line2 = "curClicks: ";
    for (int i = 0; i < 12; i++) { line2 += (allClicks[1][i] ? (clickIdxNames[i] + ", ") : ""); }
    line2 = line2.substr(0, line2.size() - 2);
    std::string line3 = "dblClicks: ";
    for (int i = 0; i < 12; i++) { line3 += (allClicks[2][i] ? (clickIdxNames[i] + ", ") : ""); }
    line3 = line3.substr(0, line3.size() - 2);
    pros::lcd::print(line, line1.c_str());
    pros::lcd::print(line + 1, line2.c_str());
    pros::lcd::print(line + 2, line3.c_str());
}

void stopMotors() {
    setDrfb(0);
    setDL(0);
    setDR(0);
    setClaw(0);
    setFlywheel(0);
    setIntake(0);
}
void stopMotorsBlock() {
    while (1) {
        setDrfb(0);
        setDL(0);
        setDR(0);
        setClaw(0);
        setFlywheel(0);
        setIntake(0);
        delay(10);
    }
}
void printPidValues() {
    printf("%.1f drfb%2d %4d/%4d fly%d %1.3f/%1.3f claw%2d %4d/%4d\n", millis() / 1000.0, (int)(getDrfbVoltage() / 1000 + 0.5), (int)drfbPid.sensVal, (int)drfbPid.target, getFlywheelVoltage(), flywheelPid.sensVal, flywheelPid.target, (int)(getClawVoltage() / 1000 + 0.5), (int)clawPid.sensVal, (int)clawPid.target);
    std::cout << std::endl;
}
extern Point g_target;
void printDrivePidValues() {
    printf("%.1f DL%d DR%d drive %3.2f/%3.2f turn %2.2f/%2.2f curve %2.2f/%2.2f x %3.2f/%3.2f y %3.2f/%3.2f a %.2f\n", millis() / 1000.0, (int)(getDLVoltage() / 100 + 0.5), (int)(getDRVoltage() / 100 + 0.5), drivePid.sensVal, drivePid.target, turnPid.sensVal, turnPid.target, curvePid.sensVal, curvePid.target, odometry.getX(), g_target.x, odometry.getY(), g_target.y, odometry.getA());
    std::cout << std::endl;
}
void printState() { printf("drfb %d claw %d ball %d %d DL %d DR %d encs %d %d %d %d\n", (int)getDrfb(), (int)getClaw(), getBallSensL(), getBallSensR(), (int)getDL(), (int)getDR(), (int)mtr1.get_position(), (int)mtr2.get_position(), (int)mtr4.get_position(), (int)mtr5.get_position()); }

void setDrfbParams(bool auton) {
    if (auton) {
        drfbPid.kp = 45.0;
        drfbPid.ki = 0.5;
        drfbPid.unwind = 20;
        drfbPid.iActiveZone = 150;
        drfbPid.maxIntegral = 3000;
        drfbPid.kd = 4500;
    } else {
        drfbPid.kp = 7.0;
        drfbPid.ki = 0.0;
        drfbPid.kd = 0.0;
    }
}
void setDriveSlew(bool auton) {
    DLSlew.slewRate = 70;  // 120
    DRSlew.slewRate = 70;  // 120
}
void setup() {
    static bool first = true;
    if (!first) {
        printf("setup already.\n");
        return;
    } else {
        printf("setting up...\n");
    }
    flywheelSlew.slewRate = 999999;  // 60;
    flywheelPid.kp = 1000.0;         // 3500
    flywheelPid.kd = 250000.0;       // 300000
    flywheelPid.dInactiveZone = 0.0;
    flywheelPid.DONE_ZONE = 0.1;
    flywheelPid.derivativeUpdateInterval = 1;
    flySaver.setConstants(1, 1, 0, 0);

    intakeSlew.slewRate = 200;
    intakeSaver.setConstants(.15, 0.3, 0.05, 0.2);

    clawPid.kp = 90.0;
    clawPid.ki = 0.03;
    clawPid.kd = 2500.0;
    clawPid.iActiveZone = 300;
    clawPid.maxIntegral = 4000;
    clawPid.unwind = 0;
    clawSlew.slewRate = 200;
    clawSaver.setConstants(0.5, .3, 0.3, .15);

    drfbSlew.slewRate = 99999;
    setDrfbParams(true);
    drfbSaver.setConstants(0.15, 0.6, 0.01, 0.1);
    drfbPid.DONE_ZONE = 100;
    drfbPid.target = drfbPos0;

    setDriveSlew(false);
    DLPid.kp = DRPid.kp = 900;
    DLPid.kd = DRPid.kd = 25000;
    DLPid.DONE_ZONE = DRPid.DONE_ZONE = 1.5;
    dlSaver.setConstants(1, 1, 0, 0);
    drSaver.setConstants(1, 1, 0, 0);

    drivePid.kp = 2000;
    drivePid.ki = 30;
    drivePid.iActiveZone = 2;
    drivePid.maxIntegral = 5000;
    drivePid.kd = 110000;
    drivePid.DONE_ZONE = 3.0;
    turnPid.kp = 28000;
    turnPid.ki = 200;
    turnPid.kd = 2000000;
    turnPid.iActiveZone = 0.1;
    turnPid.unwind = 0;
    turnPid.maxIntegral = 5000;
    turnPid.DONE_ZONE = PI / 20;

    curvePid.kp = 28000;
    curvePid.ki = 200;
    curvePid.kd = 2000000;
    curvePid.unwind = 0;
    curvePid.iActiveZone = PI / 18;
    curvePid.maxIntegral = 5000;

    drfbPid.target = drfbPos0;
    clawPid.target = 0;
    flywheelPid.target = 0;

    // ballSensL->calibrate(); fix this: calibrate in a seperate thread
    // ballSensR->calibrate();
    int t0 = millis();
    while (millis() - t0 < 800) { int n = getDL() + getDR() + getDS(); }
    first = false;
}

void morningRoutine() {
    static bool first = true;
    if (first) {
        printf("good morning.\n");
    } else {
        printf("its not the morning any more...\n");
        return;
    }
    while (!ctlr.get_digital(DIGITAL_R1) && !ctlr.get_digital(DIGITAL_R2)) {
        pros::lcd::print(1, "drfb, claw down. then press R");
        delay(10);
    }
    int t0 = millis();
    while (millis() - t0 < 600) {
        setDrfbDumb(-6000);
        setClawDumb(-2500);
    }
    drfbIMEBias = -getDrfb() - 65;
    clawOpctl::bias = -getClaw() - 70;
    setDrfb(0);
    setClaw(0);
    pros::lcd::print(1, "calibration complete");
    pros::lcd::print(2, "claw: %f", getClaw());
    pros::lcd::print(3, "drfb: %f", getDrfb());
    first = false;
}