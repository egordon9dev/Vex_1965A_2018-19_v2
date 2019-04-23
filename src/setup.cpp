#include "setup.hpp"
#include <cassert>
#include <deque>
#include <string>
#include "MotorSaver.hpp"
#include "Point.hpp"
#include "main.h"
#include "pid.hpp"

// motors
pros::Motor mtr3(19);  // intake
pros::Motor mtr6(17);  // flywheel
pros::Motor mtr7(14);  // drfb
pros::Motor mtr8(3);   // claw
// gnd: 8
/* bad ports:
5,
15(claw)
18(flywheel: during practice),
13(claw: during practice),
12 (claw: first time turning on the robot in the morning)

1.0.7 update
11 (drfb)
16
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
pros::ADILineSensor* ballSensL;
pros::ADILineSensor* ballSensR;
// pros::ADIEncoder* perpindicularWheelEnc;
pros::ADIEncoder* DLEnc;
pros::ADIEncoder* DREnc;
pros::ADIGyro* gyro;
pros::Vision* vision;

//----------- Constants ----------------
const int driveTurnLim = 9000;

const int drfbMaxPos = 2390, drfbPos0 = -60, drfbMinPos = -80, drfbPos1 = 1180, drfbPos1Plus = 1400, drfbPos2 = 1780, drfbPos2Plus = 2250;
const int drfbMinClaw0 = 350, drfbMaxClaw0 = 640, drfbMinClaw1 = 1087, drfb18Max = 350, drfbPosCloseIntake = 200, drfbPosScrape = 300, drfbPosAboveScrape = 450;
const int drfbHoldPwr = -1500;

double sShotSpeed = 2.97;
double fw_a4_middleFlag = 3.0;
double fw_a4_sideFlag = 3.0;

const int intakeOneShotTicks = 600, intakeOneShotTicksTop = 350;

const int dblClickTime = 450, claw0 = -10, claw180 = 1360;
const double /*ticksPerInch = 52.746, */ ticksPerInchADI = 35.2426, ticksPerRadian = 368.309;
const double PI = 3.14159265358979323846;
const int BIL = 1000000000, MIL = 1000000;

double getGyro() { return -gyro->get_value() / 10.0 * (PI / 180.0); }

/*
 #### ##    ## ########    ###    ##    ## ########
  ##  ###   ##    ##      ## ##   ##   ##  ##
  ##  ####  ##    ##     ##   ##  ##  ##   ##
  ##  ## ## ##    ##    ##     ## #####    ######
  ##  ##  ####    ##    ######### ##  ##   ##
  ##  ##   ###    ##    ##     ## ##   ##  ##
 #### ##    ##    ##    ##     ## ##    ## ########
*/
namespace intake {
int requestedVoltage = 0;
int altT0;
double target;
int wait;
double posBias = 0.0;
void init(double tgt, int w) {
    posBias -= getIntakePos();
    target = tgt;
    wait = w;
    intakePid.doneTime = BIL;
}
}  // namespace intake
void pidIntakeInit(double target, int wait) { intake::init(target, wait); }
double getIntakePos() { return mtr3.get_position() + intake::posBias; }
double getIntakeVel() { return mtr3.get_actual_velocity() * (3.1 / 200.0); }
bool pidIntake() {
    using intake::target;
    using intake::wait;
    intakePid.sensVal = getIntakePos();
    intakePid.target = target;
    setIntake(lround(intakeSlew.update(intakePid.update())));
    return millis() - intakePid.doneTime > wait;
}
void setIntake(int n) {  // +: front, -: back
    n = clamp(n, -12000, 12000);
    // n = intakeSlew.update(n);
    n = intakeSaver.getPwr(n, getIntakeVel());
    mtr3.move_voltage(n);
    intake::requestedVoltage = n;
}
int getIntakeVoltage() { return intake::requestedVoltage; }
void setIntake(IntakeState is) {
    static IntakeState prev = IntakeState::NONE;
    if (is == IntakeState::NONE) {
        setIntake(0);
    } else if (is == IntakeState::FRONT) {
        setIntake(12000);
    } else if (is == IntakeState::FRONT_HOLD) {
        setIntake(100);
    } else if (is == IntakeState::BACK) {
        setIntake(-6000);
    } else if (is == IntakeState::BACK_SLOW) {
        setIntake(-2000);
    } else if (is == IntakeState::ALTERNATE) {
        setIntake(0);
    }
    prev = is;
}
int getBallSensTop() { return ballSensL->get_value(); }
int getBallSensBtm() { return ballSensR->get_value(); }
bool isTopBallIn() { return getBallSensTop() < 1000; }
bool isBtmBallIn() { return getBallSensBtm() < 1370; }

//----------- DRFB functions ---------
int drfb_requested_voltage = 0;
int drfbPowerLimit = 3500;
int drfbFullRangePowerLimit = 12000;
void setDrfb(int n) {
    n = clamp(n, -drfbFullRangePowerLimit, drfbFullRangePowerLimit);
    if (getDrfb() < drfbMinPos + 150 && n < -drfbPowerLimit) n = -drfbPowerLimit;
    if (getDrfb() > drfbMaxPos - 150 && n > drfbPowerLimit) n = drfbPowerLimit;
    n = drfbSlew.update(n);
    n = drfbSaver.getPwr(n, getDrfbVel());
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
void setDrfbDull(int n) {
    n = drfbSlew.update(n);
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
void setDrfbDumb(int n) {
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
double drfbIMEBias = 0;
double getDrfb() { return drfbIMEBias + mtr7.get_position(); }
double getDrfbVel() { return mtr7.get_actual_velocity() * (3.1 / 200.0); }
int getDrfbVoltage() { return drfb_requested_voltage; }
int getDrfbCurrent() { return mtr7.get_current_draw(); }
int drfbPidBias = 0;
bool pidDrfb(double pos, int wait) {
    drfbPid.target = pos;
    drfbPid.sensVal = getDrfb();
    int out = drfbPid.update();
    setDrfb(drfbPidBias + out);
    if (drfbPid.doneTime + wait < millis()) return true;
}
void pidDrfb() { pidDrfb(drfbPid.target, 9999999); }

/*
  ######  ##          ###    ##      ##
 ##    ## ##         ## ##   ##  ##  ##
 ##       ##        ##   ##  ##  ##  ##
 ##       ##       ##     ## ##  ##  ##
 ##       ##       ######### ##  ##  ##
 ##    ## ##       ##     ## ##  ##  ##
  ######  ######## ##     ##  ###  ###
*/

namespace clawOpctl {
double bias = 0.0;
}
int claw_requested_voltage = 0;

void setClaw(int n) {
    n = n + (int)(3 * sin(millis() * 2 * PI / 250));
    n = clawSaver.getPwr(n, getClawVel());
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
double getClawVel() { return mtr8.get_actual_velocity() * (3.1 / 200.0); }
int getClawVoltage() { return claw_requested_voltage; }
int getClawMeasuredVoltage() { return mtr8.get_voltage(); }
bool pidClaw(double a, int wait) {
    clawPid.sensVal = getClaw();
    clawPid.target = a;
    setClaw(clawPid.update());
    return clawPid.doneTime + wait < millis();
}
void pidClaw() { pidClaw(clawPid.target, 999999); }

/*
 ######## ##       ##    ## ##      ## ##     ## ######## ######## ##
 ##       ##        ##  ##  ##  ##  ## ##     ## ##       ##       ##
 ##       ##         ####   ##  ##  ## ##     ## ##       ##       ##
 ######   ##          ##    ##  ##  ## ######### ######   ######   ##
 ##       ##          ##    ##  ##  ## ##     ## ##       ##       ##
 ##       ##          ##    ##  ##  ## ##     ## ##       ##       ##
 ##       ########    ##     ###  ###  ##     ## ######## ######## ########
*/
namespace flywheel {
int requestedVoltage = 0;
double target = 0;
int wait = 999999;
double pidZone = 0.1;
void init(double t, double pz, int w) {
    target = t;
    wait = w;
    pidZone = pz;
    flywheelPid.doneTime = BIL;
}
}  // namespace flywheel
void setFlywheel(int n) {
    // add some flicker to wakeup the motor
    n = n + (int)(3 * sin(millis() * 2 * PI / 250));
    n = clamp(n, 0, 12000);
    // n = flywheelSlew.update(n);
    // n = flySaver.getPwr(n, getFlywheel());
    mtr6.move_voltage(n);
    flywheel::requestedVoltage = n;
}
double getFlywheel() { return mtr6.get_position(); }
double getFlywheelFromMotor() { return 3.1 / 200.0 * mtr6.get_actual_velocity(); }
int getFlywheelVoltage() { return flywheel::requestedVoltage; }
int getFlywheelMeasuredVoltage() { return mtr6.get_voltage(); }

double FWSpeeds[][2] = {{0, 0}, {2.8, 11000}, {2.9, 11000}, {2.94, 11150}, {3.0, 10800}, {3.06, 11600}, {3.1, 11700}};
void pidFlywheelInit(double speed, double pidZone, int wait) { flywheel::init(speed, pidZone, wait); }
bool pidFlywheel() {
    static std::deque<int> pwrs;
    static bool pidShutdown = false;
    double speed = flywheel::target;
    static double prevSpeed = 0.0;
    static int prevT = 0;
    static double output = 0.0;
    static bool crossedTarget = false;
    static int dir = 1;
    static int prevUpdateT = -9999;
    static double sumVel = 0.0;
    static int numVel = 0;
    static int crossedTargetT = 0;
    static double filtD = 0.0;
    static double prevSensVal = 0;
    if (fabs(speed - prevSpeed) > 0.01) {
        printf("\n\nFlywheelPid New Target\n");
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
        flywheelPid.target = speed;

        sumVel += getFlywheelFromMotor();
        numVel++;
        bool updateNow = false;
        if (millis() - prevUpdateT > 50) {
            // average sensor reading during the the update interval
            flywheelPid.sensVal = sumVel / numVel;
            updateNow = true;
        }

        if (crossedTarget) {
            // printf("{fw1 cur: %1.3f, prev: %1.3f}\n", flywheelPid.sensVal, prevSensVal);
            // after shooting a ball, the flywheel slows down a lot
            if (flywheelPid.sensVal < prevSensVal - 0.1 || flywheelPid.sensVal < flywheelPid.target - 0.3) {
                dir = 1;
                output = 0.0;
                crossedTarget = false;
                printf("\n---------------- Flywheel PID Reset -----------------\n\n");
            }
        }
        if (!crossedTarget) {
            pidShutdown = false;
            if (dir == 1) {
                if (flywheelPid.sensVal > flywheelPid.target - flywheel::pidZone) {
                    crossedTarget = true;
                    crossedTargetT = millis();
                } else {
                    setFlywheel(12000);
                }
            } else {
                if (flywheelPid.sensVal < flywheelPid.target + flywheel::pidZone) {
                    crossedTarget = true;
                    crossedTargetT = millis();
                } else {
                    setFlywheel(0);
                }
            }
        }
        if (updateNow) {
            // reset for next time
            sumVel = 0.0;
            numVel = 0;
            prevUpdateT = millis();
            flywheelPid.update();
            double k = 0.9;
            filtD = k * filtD + (1 - k) * flywheelPid.deriv;
            double deltaOutput = flywheelPid.prop + filtD;
            if (crossedTarget) {
                if (millis() - crossedTargetT > 50000000) pidShutdown = true;
                if (pidShutdown && pwrs.size() > 0) {
                    int sum = 0;
                    for (const auto& p : pwrs) { sum += p; }
                    int avgPwr = sum / pwrs.size();
                    setFlywheel(avgPwr);
                    // printf("settled at %d", avgPwr);
                } else {
                    output += deltaOutput;
                    output = clamp(output, -6000.0, 6000.0);
                    int curPwr = bias + lround(output);
                    setFlywheel(curPwr);
                    pwrs.push_back(curPwr);
                    while (pwrs.size() > 10) pwrs.pop_front();
                }
            }
            // printf("{ flywheelPid, output: %d, crossedTarget: %s, prevSensVal: %1.3f } ", lround(output), crossedTarget ? "True" : "False", prevSensVal);
            prevSensVal = flywheelPid.sensVal;
        }
    }
    return millis() - flywheelPid.doneTime > flywheel::wait;
}
bool isPidFlywheelDone() { return millis() - flywheelPid.doneTime > flywheel::wait; }

/*
 ##     ## ####  ######   ######
 ###   ###  ##  ##    ## ##    ##
 #### ####  ##  ##       ##
 ## ### ##  ##   ######  ##
 ##     ##  ##        ## ##
 ##     ##  ##  ##    ## ##    ##
 ##     ## ####  ######   ######
*/

int clamp(int n, int a, int b) { return n < a ? a : (n > b ? b : n); }
double clamp(double n, double a, double b) { return n < a ? a : (n > b ? b : n); }
Point polarToRect(double mag, double angle) {
    Point p(mag * cos(angle), mag * sin(angle));
    return p;
}

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
int millis() { return pros::millis(); }
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
        stopMotors();
        delay(10);
    }
}
extern Point g_target;
void printPidValues() {
    printf("drfb%2d %4d/%4d fly%d %1.3f/%1.3f e%1.3f clw%2d|%5dv %4d|%4d/%4d intk%2d %4d/%4d ball b %d %d t %d %d  vel %+1.1f drv %+3.2f/%+3.1f DL%+5d %+2.1f/%+2.1f DR%+5d %+2.1f/%+2.1f trn %+2.2f/%+2.1f x %+3.2f/%+3.1f y %+3.2f/%+3.1f a %+1.2f\n", (int)(getDrfbVoltage() / 1000 + 0.5), (int)getDrfb(), (int)drfbPid.target, getFlywheelVoltage(), flywheelPid.sensVal, flywheelPid.target, fabs(flywheelPid.sensVal - flywheelPid.target), (int)(getClawVoltage() / 1000 + 0.5), getClawMeasuredVoltage(), lround(clawPid.sensVal), lround(getClaw()), lround(clawPid.target), (int)(getIntakeVoltage() / 1000 + 0.5), (int)intakePid.sensVal, (int)intakePid.target, isBtmBallIn() ? 1 : 0, getBallSensBtm(), isTopBallIn() ? 1 : 0, getBallSensTop(), getDriveVel(), drivePid.sensVal, drivePid.target, getDLVoltage(), DLPid.sensVal, DLPid.target, getDRVoltage(), DRPid.sensVal, DRPid.target, turnPid.sensVal, turnPid.target, odometry.getX(), g_target.x, odometry.getY(), g_target.y, odometry.getA());
}
void printDrivePidValues() { printf("DL%+5d DR%+5d (%+5d %+5d %+5d) vel %+1.3f drive %+3.2f/%+3.2f DL %+2.1f/%+2.1f DR %+2.1f/%+2.1f turn %+2.2f/%+2.2f curve %+2.2f/%+2.2f x %+3.2f/%+3.2f y %+3.2f/%+3.2f a %+1.2f\n", getDLVoltage(), getDRVoltage(), (int)getDL(), (int)getDR(), (int)getDS(), getDriveVel(), drivePid.sensVal, drivePid.target, DLPid.sensVal, DLPid.target, DRPid.sensVal, DRPid.target, turnPid.sensVal, turnPid.target, curvePid.sensVal, curvePid.target, odometry.getX(), g_target.x, odometry.getY(), g_target.y, odometry.getA()); }
void printPidSweep() { printf("DL%d %.1f/%.1f DR%d %.1f/%.1f\n", getDLVoltage, DLPid.sensVal, DLPid.target, getDRVoltage, DRPid.sensVal, DRPid.target); }
void odoTaskRun(void* param) {
    while (true) {
        odometry.update();
        bentOdo.update();
        delay(3);
    }
}
void startOdoTask() {
    std::string s("param");
    pros::Task odoTask(odoTaskRun, &s);
}

void opctlDrive(int driveDir) {
    static bool prevStopped = false;
    static int stopT = 0;
    const int nVels = 2;
    static std::deque<double> dlVels, drVels;
    dlVels.push_back(getDLVel());
    while (dlVels.size() > nVels) { dlVels.pop_front(); }
    drVels.push_back(getDRVel());
    while (drVels.size() > nVels) { drVels.pop_front(); }
    double dlVelAvg = 0, drVelAvg = 0;
    for (const auto& v : dlVels) { dlVelAvg += v; }
    for (const auto& v : drVels) { drVelAvg += v; }
    dlVelAvg /= nVels;
    drVelAvg /= nVels;
    static double dlOut = 0, drOut = 0;
    int trn = lround(ctlr.get_analog(ANALOG_RIGHT_X));
    int drv = lround(driveDir * ctlr.get_analog(ANALOG_LEFT_Y));
    if (abs(drv) < 12) drv = 0;
    if (abs(trn) < 12) trn = 0;
    drv = lround(drv * 12000.0 / 127.0);
    trn = lround(trn * 12000.0 / 127.0);
    if (abs(drv) < 4000) trn = clamp(trn, -driveTurnLim, driveTurnLim);
    bool stopped = drv == 0 && trn == 0;
    if (stopped && !prevStopped) {
        Point pos = odometry.getPos();
        dlOut = drOut = 0;
        stopT = millis();
    }
    if (stopped) {
        const int breakPwr = 3000;
        driveLim = breakPwr;
        if (millis() - stopT < 500) {
            double k = 1000;
            dlOut = -k * getDLVel();
            drOut = -k * getDRVel();
        } else {
            //---------------   antipush   ------------------
            if (getDrfb() < drfb18Max + 50) {
                double pushVel = 0.35, breakCutoffVel = 0.05;
                // positive push
                if (dlVelAvg > pushVel && dlOut <= 0) {
                    printf("+ break ON\n");
                    dlOut = -breakPwr;
                } else if (dlVelAvg < -breakCutoffVel && dlOut < -500) {
                    printf("+ break OFF\n");
                    dlOut = 500;
                }
                if (drVelAvg > pushVel && drOut <= 0) {
                    drOut = -breakPwr;
                } else if (drVelAvg < -breakCutoffVel && drOut < -500) {
                    drOut = 500;
                }
                // negative push
                if (dlVelAvg < -pushVel && dlOut >= 0) {
                    printf("- break ON\n");
                    dlOut = breakPwr;
                } else if (dlVelAvg > breakCutoffVel && dlOut > 500) {
                    printf("- break OFF\n");
                    dlOut = -500;
                }
                if (drVelAvg < -pushVel && drOut >= 0) {
                    drOut = breakPwr;
                } else if (drVelAvg > breakCutoffVel && drOut > 500) {
                    drOut = -500;
                }
            } else {
                dlOut = drOut = 0;
            }
        }
        setDL(dlOut);
        setDR(drOut);
    } else {
        setDL(drv + trn);
        setDR(drv - trn);
    }
    prevStopped = stopped;
}
void driveToCap(bool red, int pwr, double offset) {
    pros::vision_object_s_t objs[10];
    int nObjs = vision->read_by_sig(0, red ? 1 : 2, 10, objs);
    int out = 0;
    turnPid.target = 0;
    if (nObjs > 0 && nObjs != PROS_ERR) {
        double smallestDist = BIL;
        for (int i = 0; i < nObjs; i++) {
            double x = objs[i].left_coord, y = objs[i].top_coord, w = objs[i].width, h = objs[i].height;
            x += w / 2;
            y += h / 2;
            double dist = sqrt(pow(x - 175 - offset, 2) + pow(170 - y, 2));
            double pos = clamp((x - 175 - offset) * 0.0045, -PI / 4, PI / 4);
            if (dist < smallestDist) {
                smallestDist = dist;
                turnPid.sensVal = pos;
            }
        }
        out = turnPid.update();
        if (pwr >= 6000) {
            out *= 1.7;
        } else {
            out *= 0.9;
        }
    } else {
        out = 0;
    }
    setDL(pwr - out);
    setDR(pwr + out);
}
/*
  ######  ######## ######## ##     ## ########
 ##    ## ##          ##    ##     ## ##     ##
 ##       ##          ##    ##     ## ##     ##
  ######  ######      ##    ##     ## ########
       ## ##          ##    ##     ## ##
 ##    ## ##          ##    ##     ## ##
  ######  ########    ##     #######  ##
*/

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
    if (auton) {
        DLSlew.slewRate = DRSlew.slewRate = 120;
    } else {
        DLSlew.slewRate = DRSlew.slewRate = 120;
    }
}
void setup() {
    static bool first = true;
    if (!first) {
        printf("setup already.\n");
        return;
    } else {
        printf("setting up...\n");
    }
    // 2 fw + mesh,rubber bands: kp=2000 kd=700k:     2.9-2.43  2.12-1.73
    // 2 fw + rubber bands:                             2.9-2.54  2.492-2.203
    // 1 fw: kp=700 kd=180k
    // complex fw: kp=1000, kd=200000
    flywheelPid.kp = 1000.0;
    flywheelPid.ki = 0;
    flywheelPid.kd = 150000;  // 150k
    flywheelPid.DONE_ZONE = 0.1;

    intakePid.kp = 100;
    intakePid.ki = 0.05;
    intakePid.kd = 5000;
    intakePid.maxIntegral = 4000;
    intakePid.iActiveZone = 300;
    intakePid.unwind = 0;
    intakePid.DONE_ZONE = 50;
    intakeSaver.setConstants(6000, 3500, 0.5, 0.2);

    clawPid.kp = 70.0;
    clawPid.ki = 0.03;
    clawPid.kd = 2500.0;
    clawPid.iActiveZone = 300;
    clawPid.maxIntegral = 4000;
    clawPid.unwind = 0;
    clawSaver.setConstants(6000, 3500, 0.5, 0.2);

    setDrfbParams(true);
    drfbSaver.setConstants(6000, 3500, 0.5, 0.2);
    drfbPid.DONE_ZONE = 100;
    drfbPid.target = drfbPos0;
    drfbSlew.slewRate = 200;

    setDriveSlew(false);
    dlSaver.setConstants(6000, 3500, 0.5, 0.2);
    drSaver.setConstants(6000, 3500, 0.5, 0.2);

    drivePid.kp = 900;    // 1100
    drivePid.ki = 3;      // 3
    drivePid.kd = 70000;  // 80k
    drivePid.iActiveZone = 3;
    drivePid.maxIntegral = 4000;
    drivePid.DONE_ZONE = 1;
    DRPid = DLPid = drivePid;

    turnPid.kp = 12000;
    turnPid.ki = 80;
    turnPid.kd = 2000000;
    turnPid.iActiveZone = 0.25;
    turnPid.unwind = 0.0;
    turnPid.maxIntegral = 5000;
    turnPid.DONE_ZONE = 0.15;

    sTurnPid.kp = 59000;
    sTurnPid.ki = 500;
    sTurnPid.kd = 4000000;
    sTurnPid.iActiveZone = 0.1;
    sTurnPid.unwind = 0.003;
    sTurnPid.maxIntegral = 5000;
    sTurnPid.DONE_ZONE = PI / 20;

    curvePid.kp = 55000;
    curvePid.kd = 5500000;

    curveVelPid.kp = 10000000;

    drfbPid.target = drfbPos0;
    clawPid.target = claw0;
    flywheelPid.target = 0;

    ballSensL = new pros::ADILineSensor(7);
    ballSensR = new pros::ADILineSensor(8);
    // perpindicularWheelEnc = new pros::ADIEncoder(3, 4, false);
    DLEnc = new pros::ADIEncoder(1, 2, false);
    DREnc = new pros::ADIEncoder(5, 6, false);
    gyro = new pros::ADIGyro(3, 1);
    vision = new pros::Vision(2);
    vision->set_led(0);
    // vision = new pros::Vision(vision_port);
    // vex::vision::signature SIG_1 (1, -2379, -2025, -2202, 4005, 5441, 4723, 7, 0);
    // vex::vision::signature SIG_2 (2, 7265, 7861, 7563, -1967, -941, -1454, 7, 0);
    int t0 = millis();
    // while (millis() - t0 < 800) { int n = getDL() + getDR() + getDS();delay(10); }
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
        delay(10);
    }
    drfbIMEBias = -getDrfb() - 98;
    clawOpctl::bias = -getClaw() - 70;
    setDrfb(0);
    setClaw(0);
    pros::lcd::print(1, "Move the Drive");
    odometry.reset();
    double prevDL = 0.0, prevDR = 0.0, prevDS = 0.0;
    bool initDL = false, initDR = false, initDS = false;
    while (1) {
        // printf("%d %d %d %d %d <%d %d>\n", (int)getDL(), (int)getDR(), (int)getDS(), getBallSensTop(), getBallSensBtm(), (int)mtr1.get_position(), (int)mtr4.get_position());
        if (fabs(getDL() - prevDL) > 0.001) initDL = true;
        if (fabs(getDR() - prevDR) > 0.001) initDR = true;
        if (fabs(getDS() - prevDS) > 0.001) initDS = true;
        if (initDL && initDR) break;
        prevDL = getDL();
        prevDR = getDR();
        prevDS = getDS();
        delay(10);
    }
    first = false;
    startOdoTask();
}

/*
    ###    ##     ## ########  #######      ######  ######## ##       ########  ######  ########
   ## ##   ##     ##    ##    ##     ##    ##    ## ##       ##       ##       ##    ##    ##
  ##   ##  ##     ##    ##    ##     ##    ##       ##       ##       ##       ##          ##
 ##     ## ##     ##    ##    ##     ##     ######  ######   ##       ######   ##          ##
 ######### ##     ##    ##    ##     ##          ## ##       ##       ##       ##          ##
 ##     ## ##     ##    ##    ##     ##    ##    ## ##       ##       ##       ##    ##    ##
 ##     ##  #######     ##     #######      ######  ######## ######## ########  ######     ##
 */
bool autoSel_leftSide = false;
int autoSel_nAuton = 0;
void autoSel_update() {
    if (ctlr.get_digital_new_press(DIGITAL_LEFT)) {
        autoSel_leftSide = !autoSel_leftSide;
    } else if (ctlr.get_digital_new_press(DIGITAL_RIGHT)) {
        autoSel_nAuton++;
        if (autoSel_nAuton > 2) autoSel_nAuton = 0;
    }
    pros::lcd::print(1, " AUTON SELECT ");
    if (autoSel_nAuton == 0) {
        pros::lcd::print(2, "0)  NONE");
    } else if (autoSel_nAuton == 1) {
        pros::lcd::print(2, "1)  Near Middle");
    } else if (autoSel_nAuton == 2) {
        pros::lcd::print(2, "2)  Middle Far");
    }
    if (autoSel_nAuton == 0) {
        pros::lcd::clear_line(3);
    } else if (autoSel_leftSide) {
        pros::lcd::print(3, "Left, Red Side");
    } else {
        pros::lcd::print(3, "Right, Blue Side");
    }
    pros::lcd::print(4, "[L/R]  [ ]  [+n]");
}