#include "auton.hpp"
#include "main.h"
#include "pid.hpp"
#include "point.hpp"
#include "setup.hpp"
#include "test.hpp"
/*
Ch3         drive
Ch1         turn
R1          lift auto up
R2          lift auto down
L1          intake 1 ball
UP          shoot 1 ball, intake next ball
A, Y        flip  cap
*/

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
using pros::delay;
using std::cout;
using std::endl;
void testAuton();
void opcontrol() {
    morningRoutine();
    ctlr.clear_line(2);
    if (pros::battery::get_capacity() < 15.0) {
        for (int i = 1; i < 8; i++) {
            pros::lcd::print(i, "LOW BATTERY");
            std::cout << "LOW BATTERY" << std::endl;
        }
        return;
    }
    if (1) {
        // odometry.setA(0);
        // pidTurnInit(0.2, 9999);
        // while (1) {
        //     odometry.update();
        //     pidTurn();
        //     printDrivePidValues();
        //     delay(10);
        // }
        // testDriveMtrs();
        // while (!ctlr.get_digital(DIGITAL_B)) delay(10);
        // odometry.setA(-PI / 2);
        // odometry.setX(0);
        // odometry.setY(0);
        // odometry.reset();
        // setDriveSlew(true);
        // pidSweepInit(-5, -30, 999);
        // while (ctlr.get_digital(DIGITAL_B)) {
        //     odometry.update();
        //     pidSweep();
        //     printPidSweep();
        //     delay(10);
        // }
        // while (1) {
        //     stopMotors();
        //     delay(10);
        // }
        auton3(false);
        // int tttt = millis();
        // flywheelPid.target = 1.5;
        // while (millis() - tttt < 800) pidFlywheel();
        // initPidIntake(-600, 0);
        // while (1) {
        //     pidFlywheel();
        //     printPidValues();
        //     pidIntake();
        //     delay(10);
        // }
        printf("\nterminated\n");
        while (1) delay(1000);
        testAuton();
        odometry.setA(PI / 2);
        odometry.setX(0);
        odometry.setY(0);
        pidDriveInit(Point(0, 25), 200);
        while (!ctlr.get_digital(DIGITAL_B)) {
            odometry.update();
            printDrivePidValues();
            if (pidDrive()) break;
            delay(10);
        }
        while (1) {
            stopMotors();
            delay(10);
        }
        while (0) {
            printf("%d\n", getDrfb());
            delay(100);
        }
        auton2(true);
        while (1) delay(5000);

        flywheelPid.target = 0;
        clawPid.target = getClaw();
        drfbPid.target = getDrfb();
        int i = 0, t0, sideSign = 1, driveT = 100;
        bool drfbPidRunning = true, clawPidRunning = true;
        int lastT, autonT0 = millis();
        bool printing = false;
        Point ptA = Point(-5.3, 0), ptB;

        while (0) {
            for (int i = 0; i < 5; i++) {
                odometry.update();
                // pros::lcd::print(0, "x %f", odometry.getX());
                // pros::lcd::print(1, "y %f", odometry.getY());
                // pros::lcd::print(2, "a %f", odometry.getA());
                // printDrivePidValues();
                // printf("claw %d     drfb %d\n", (int)getClaw(), (int)getDrfb());
                pidDriveArc();
                delay(10);
            }
            printArcData();
        }
    }
    setDriveSlew(false);
    setDrfbParams(false);
    const int opcontrolT0 = millis();
    double drv[] = {0, 0};
    int prevT = 0;
    int dt = 0;
    bool prevDY = false, prevDA = false, prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false, prevX = false, prevB = false;
    int prevL2T = -9999999;
    int tDrfbOff = 0;
    bool drfbPidRunning = false;
    bool clawFlipped = false, clawInit = false;
    IntakeState intakeState = IntakeState::NONE;
    int driveDir = 1;
    int nBalls = 0;
    int intakeT0 = BIL;
    bool clawFlipRequest = false;
    int flywheelPower = 0;
    setDrfbParams(false);
    driveLim = 12000;
    double atDrfbSetp = false;
    int autoFlipI = -1, dShotI = -1;
    int dShotT0;
    int autoFlipH;
    double autoFlipDrfbTarget;
    int prevCtlrUpdateT = 0;
    bool prevIsBallIn = false;

    flywheelPid.target = 1.0;
    while (true) {
        dt = millis() - prevT;
        prevT = millis();
        pros::lcd::print(7, "%.2lfv      %d%%", pros::battery::get_voltage() / 1000.0, (int)pros::battery::get_capacity());
        // pros::lcd::print(1, "drfb %d", getDrfb());
        // pros::lcd::print(2, "ballSens %d", getBallSens());
        // pros::lcd::print(3, "claw %d", getClaw());
        // pros::lcd::print(4, "flywheel %d", getFlywheel());
        odometry.update();

        pros::lcd::print(0, "x %f", odometry.getX());
        pros::lcd::print(1, "y %f", odometry.getY());
        pros::lcd::print(2, "a %f", odometry.getA());
        pros::lcd::print(3, "L %f", getDL());
        pros::lcd::print(4, "R %f", getDR());
        pros::lcd::print(5, "S %f", getDS());
        pros::lcd::print(6, "bat: %f", pros::battery::get_capacity());
        pros::lcd::print(7, "drfb: %.2f", getDrfb());
        int driveLimFromPartner = 12000;
        if (ctlr2.get_digital(DIGITAL_R1)) {
            driveLimFromPartner = 4000;
        } else if (ctlr2.get_digital(DIGITAL_R2)) {
            driveLimFromPartner = 8000;
        }
        /*
         pros::lcd::print(3, "drfb %d", getDrfb());*/
        printPidValues();
        bool** allClicks = getAllClicks();
        bool prevClicks[12], curClicks[12], dblClicks[12];
        for (int i = 0; i < 12; i++) {
            prevClicks[i] = allClicks[0][i];
            curClicks[i] = allClicks[1][i];
            dblClicks[i] = allClicks[2][i];
        }
        // printAllClicks(5, allClicks);
        if (curClicks[ctlrIdxDown]) odometry.reset();

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) { driveDir *= -1; }
        // DRIVE
        driveLim = clamp((getDrfb() > 0.5 * (drfb18Max + drfbPos1)) ? 8500 : 12000, 0, driveLimFromPartner);
        opctlDrive(driveDir);
        // printf("%d %d\n", joy[0], joy[1]);

        // FLYWHEEL
        // ----------- Single Shot ------------
        if (dblClicks[ctlrIdxRight] && !prevClicks[ctlrIdxRight]) {  // request a double shot
            dShotI = 0;
        } else if (dblClicks[ctlrIdxDown]) {
            flywheelPid.target = 1.0;  // 0.0;
            dShotI = -1;
        } else if (curClicks[ctlrIdxDown]) {
            flywheelPid.target = 1.0;
            dShotI = -1;
        } else if (curClicks[ctlrIdxLeft]) {
            flywheelPid.target = dShotSpeed1;
            dShotI = -1;
        } else if (curClicks[ctlrIdxRight] && !prevClicks[ctlrIdxRight]) {
            flywheelPid.target = dShotSpeed2;
            dShotI = -1;
        } else if (curClicks[ctlrIdxUp]) {
            flywheelPid.target = 2.9;
            dShotI = -1;
        }
        // ---------- Double Shot -------------
        if (dShotI == 0) {
            dShotT0 = BIL;
            dShotI++;
        }
        if (dShotI == 1) {
            // load ball 1, wait for flywheel
            printf("dShot prep ball 1 ");
            flywheelPid.target = dShotSpeed2;
            if (isBallIn()) {
                intakeState = IntakeState::FRONT_SLOW;
            } else {
                intakeState = IntakeState::ALTERNATE;
            }
            if (fabs(flywheelPid.sensVal - flywheelPid.target) < 0.05 && isBallIn() && dShotT0 > millis()) { dShotT0 = millis(); }
            if (millis() - dShotT0 > 700) {
                dShotT0 = millis();
                dShotI++;
            }
        } else if (dShotI == 2) {  // shoot ball 1
            printf("dShot shoot ball 1 ");
            intakeState = IntakeState::BACK_SLOW;
            if (millis() - dShotT0 > 400) {
                intakeState = IntakeState::FRONT;
                dShotT0 = BIL;
                dShotI++;
            }
        } else if (dShotI == 3) {  // load ball 2, wait for flywheel
            printf("dShot prep ball 2 ");
            flywheelPid.target = dShotSpeed1;
            if (isBallIn()) {
                intakeState = IntakeState::FRONT_SLOW;
            } else {
                intakeState = IntakeState::ALTERNATE;
            }
            if (fabs(flywheelPid.sensVal - flywheelPid.target) < 0.05 && isBallIn() && dShotT0 > millis()) { dShotT0 = millis(); }
            if (millis() - dShotT0 > 1000) { dShotI++; }
        } else if (dShotI == 4) {  // shoot ball 2
            printf("dShot shoot ball 2 ");
            intakeState = IntakeState::BACK_SLOW;
        }
        pidFlywheel();
        // printf("{req %d actl %d}", getFlywheelVoltage(), mtr6.get_voltage());
        // drfb
        if (autoFlipI == -1) {
            clawPowerLimit = 12000;
            drfbFullRangePowerLimit = 12000;
        }
        double drfbPos = getDrfb();
        /*if (curClicks[ctlrIdxR1] && (curClicks[ctlrIdxY] || curClicks[ctlrIdxA])) {
            if (!prevClicks[ctlrIdxR1]) drfbIMEBias -= 10;
        } else if (curClicks[ctlrIdxR2] && (curClicks[ctlrIdxY] || curClicks[ctlrIdxA])) {
            if (!prevClicks[ctlrIdxR2]) drfbIMEBias += 10;
        } else */
        if (curClicks[ctlrIdxR1]) {
            drfbPidRunning = false;
            tDrfbOff = millis();
            setDrfb(12000);
            autoFlipI = -1;
            atDrfbSetp = false;
        } else if (curClicks[ctlrIdxR2]) {
            drfbPidRunning = false;
            tDrfbOff = millis();
            setDrfb(-12000);
            autoFlipI = -1;
            atDrfbSetp = false;
        } else if (curClicks[ctlrIdxY]) {
            atDrfbSetp = true;
            drfbPidRunning = true;
            drfbPid.target = drfbPos1;
            setDrfbParams(true);
        } else if (curClicks[ctlrIdxA]) {
            atDrfbSetp = true;
            drfbPidRunning = true;
            drfbPid.target = drfbPos2;
            setDrfbParams(true);
        } else if (autoFlipI > -1) {  // fix this : add ability to autoflip on the floor
            if (autoFlipI == 0) {
                printf("auto flip step 0 ");
                atDrfbSetp = false;
                if (getDrfb() < drfbMinClaw0) {
                    autoFlipH = 0;
                    drfbPidRunning = true;
                    setDrfbParams(true);
                    autoFlipI++;
                } else if (fabs(getDrfb() - drfbPos1) < 100) {
                    autoFlipH = 1;
                    drfbPidRunning = true;
                    setDrfbParams(true);
                    autoFlipI++;
                } else if (fabs(getDrfb() - drfbPos2) < 100) {
                    autoFlipH = 2;
                    drfbPidRunning = true;
                    setDrfbParams(true);
                    autoFlipI++;
                } else {
                    drfbFullRangePowerLimit = 12000;
                    autoFlipI = -1;
                }
            } else if (autoFlipI == 1) {
                printf("auto flip step 1 ");
                if (autoFlipH == 0) {
                    drfbPidBias = 4000;
                    drfbPid.target = drfb18Max;
                } else if (autoFlipH == 1) {
                    drfbPidBias = 4000;
                    drfbPid.target = drfbPos1Plus;
                } else if (autoFlipH == 2) {
                    drfbPidBias = 4000;
                    drfbPid.target = drfbMaxPos;
                }
                if (getDrfb() > drfbPid.target - 150) {
                    clawFlipRequest = true;
                    autoFlipI++;
                }
            } else if (autoFlipI == 2) {
                printf("auto flip step 2 ");
                if (!clawFlipRequest && fabs(getClaw() - clawPid.target) < claw180 * (autoFlipH == 2 ? 0.3 : 0.45)) {
                    drfbPidBias = 0;
                    drfbPid.target = drfbPos0;
                    if (autoFlipH == 1) {
                        drfbPid.target = drfbPos1;
                        atDrfbSetp = true;
                    } else if (autoFlipH == 2) {
                        drfbPid.target = drfbPos2;
                        drfbFullRangePowerLimit = 3000;
                        atDrfbSetp = true;
                    }
                    autoFlipI++;
                }
            } else if (autoFlipI == 3) {
                printf("auto flip step 3 ");
                if (fabs(getDrfb() - drfbPid.target) < 100) {
                    autoFlipI = -1;
                    drfbFullRangePowerLimit = 12000;
                }
            }
        } else if (millis() - tDrfbOff > 130 && millis() - opcontrolT0 > 300) {
            if (!drfbPidRunning) {
                drfbPidRunning = true;
                drfbPid.target = getDrfb();
                setDrfbParams(false);
            }
        } else if (!drfbPidRunning) {
            setDrfb(0);
        }
        if (drfbPidRunning) pidDrfb();

        // CLAW
        if (curClicks[ctlrIdxX] && !prevClicks[ctlrIdxX]) {
            if (atDrfbSetp && (fabs(getDrfb() - drfbPos1) < 100 || fabs(getDrfb() - drfbPos2) < 100) || getDrfb() < drfbMinClaw0) {
                // request an auto-flip
                if (autoFlipI == -1) autoFlipI = 0;
            } else if (autoFlipI == -1) {
                clawFlipRequest = true;
            }
        }
        if (clawFlipRequest && millis() - opcontrolT0 > 300) {
            // move the drfb to within an acceptable range
            if (getDrfb() < drfbMinClaw0) {
                drfbPidRunning = true;
                setDrfbParams(true);
                drfbPid.target = drfb18Max;
            } else if (getDrfb() > drfbMaxClaw0 && getDrfb() < drfbMinClaw1) {
                drfbPidRunning = true;
                setDrfbParams(true);
                drfbPid.target = drfbMinClaw1 + 50;
            }
            // fullfill the request if the drfb is within an acceptable range
            if ((getDrfb() > drfbMinClaw0 && getDrfb() < drfbMaxClaw0) || getDrfb() > drfbMinClaw1) {
                clawFlipped = !clawFlipped;
                clawFlipRequest = false;
            }
        }
        clawPid.target = clawFlipped ? claw180 : 0;
        clawPid.sensVal = getClaw();
        setClaw(clamp(clawPid.update(), -12000.0, 12000.0));

        // INTAKE
        /*
        ------ intended functionality ------
        intake FRONT:   grab two balls
        intake BACK:    load balls
        intake BACK:    fire first ball
        intake BACK:    fire second ball
        */
        if (dblClicks[ctlrIdxL2]) {
            if (dShotI == 4) dShotI = -1;
            intakeState = IntakeState::NONE;
        } else if (curClicks[ctlrIdxL2]) {
            if (dShotI == 4) dShotI = -1;
            intakeState = IntakeState::FRONT;
        } else if (curClicks[ctlrIdxL1]) {
            intakeState = IntakeState::BACK_SLOW;
        } else if (dShotI == -1) {
            if (intakeState == IntakeState::BACK_SLOW) { intakeState = IntakeState::ALTERNATE; }
            // prevent stall
            /*if (intakeState == IntakeState::FRONT && intakeSaver.isPwr(0.7) && !intakeSaver.isFaster(0.2)) { intakeState = IntakeState::ALTERNATE; }*/
        }
        setIntake(intakeState);
        if (millis() - prevCtlrUpdateT > 150) {
            bool curIsBallIn = isBallIn();
            if (curIsBallIn) {
                if (prevIsBallIn) {
                    ctlr.print(2, 0, "--- Ball In ---");
                } else {
                    ctlr.rumble(" .");
                }
            } else {
                ctlr.print(2, 0, "                ");
            }
            prevIsBallIn = curIsBallIn;
            prevCtlrUpdateT = millis();
        }
        delete[] allClicks[0];
        delete[] allClicks[1];
        delete[] allClicks[2];
        delete[] allClicks;
        pros::delay(10);
    }
    delete ballSensL;
    delete ballSensR;
    delete perpindicularWheelEnc;
    delete DLEnc;
    delete DREnc;
}