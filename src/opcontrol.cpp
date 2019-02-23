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
bool robotInit = false;
void opcontrol() {
    if (!robotInit) {
        morningRoutine();
        pros::lcd::print(6, "R1 to confirm");
        while (!ctlr.get_digital(DIGITAL_R1)) {
            autoSel_update();
            delay(5);
        }
        robotInit = true;
    }
    pros::lcd::print(4, "   READY    ");
    ctlr.clear_line(2);
    if (pros::battery::get_capacity() < 15.0) {
        for (int i = 1; i < 8; i++) {
            pros::lcd::print(i, "LOW BATTERY");
            std::cout << "LOW BATTERY" << std::endl;
        }
        return;
    }
    if (1) {
        // odometry.setA(-PI / 2);
        // odometry.setX(0);
        // odometry.setY(0);
        // odometry.reset();
        // while (1) {
        //     printf("%d %d %d %.2f %.2f %.2f\n", (int)getDL(), (int)getDR(), (int)getDS(), odometry.getX(), odometry.getY(), odometry.getA());
        //     delay(20);
        // }
        // odometry.setA(0);
        // pidTurnInit(0.2, 9999);
        // while (1) {
        //     pidTurn();
        //     printDrivePidValues();
        //     delay(10);
        // }
        // testDriveMtrs();
        // while (!ctlr.get_digital(DIGITAL_B)) delay(10);
        // setDriveSlew(true);
        // pidDriveLineInit(Point(0, 0), Point(0, 36), true, 0.1, 2000);
        // while (!ctlr.get_digital(DIGITAL_B)) {
        //     pidDriveLine();
        //     printDrivePidValues();
        //     delay(10);
        // }
        // while (1) {
        //     stopMotors();
        //     delay(10);
        // }
        auton3(true);
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
    }
    setDriveSlew(false);
    setDrfbParams(false);
    const int opcontrolT0 = millis();
    double drv[] = {0, 0};
    bool prevDY = false, prevDA = false, prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false, prevX = false, prevB = false;
    int tDrfbOff = 0;
    bool drfbPidRunning = false;
    bool clawFlipped = false;
    IntakeState intakeState = IntakeState::NONE;
    bool intakeRunning = true;
    int driveDir = 1;
    bool clawFlipRequest = false;
    setDrfbParams(false);
    driveLim = 12000;
    double atDrfbSetp = false;
    int autoFlipI = -1, dShotI = -1, prevDShotI = -1;

    int autoFlipH;
    int prevCtlrUpdateT = 0;
    bool prevIsBallIn = false;

    pidFlywheelInit(1.0, 9999);
    while (true) {
        pros::lcd::print(0, "x %f", odometry.getX());
        pros::lcd::print(1, "y %f", odometry.getY());
        pros::lcd::print(2, "a %f", odometry.getA());
        pros::lcd::print(3, "L %f", getDL());
        pros::lcd::print(4, "R %f", getDR());
        pros::lcd::print(5, "S %f", getDS());
        pros::lcd::print(6, "bat: %f", pros::battery::get_capacity());
        pros::lcd::print(7, "drfb: %.2f", getDrfb());
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

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) { driveDir *= -1; }
        // DRIVE
        driveLim = (getDrfb() > 0.5 * (drfb18Max + drfbPos1)) ? 8500 : 12000;
        opctlDrive(driveDir);
        // printf("%d %d\n", joy[0], joy[1]);

        // FLYWHEEL
        // ----------- Single Shot ------------
        if (dblClicks[ctlrIdxRight] && !prevClicks[ctlrIdxRight]) {  // request a double shot
            dShotI = 0;
        } else if (dblClicks[ctlrIdxDown]) {
            pidFlywheelInit(1.0, 700);  // 0.0;
            dShotI = -1;
        } else if (curClicks[ctlrIdxDown]) {
            pidFlywheelInit(1.0, 700);
            dShotI = -1;
        } else if (curClicks[ctlrIdxLeft]) {
            pidFlywheelInit(dShotSpeed1, 700);
            dShotI = -1;
        } else if (curClicks[ctlrIdxRight] && !prevClicks[ctlrIdxRight]) {
            if (fabs(flywheelPid.target - dShotSpeed2) > 0.001) pidFlywheelInit(dShotSpeed2, 700);
            dShotI = -1;
        } else if (curClicks[ctlrIdxUp]) {
            pidFlywheelInit(sShotSpeed, 700);
            dShotI = -1;
        }
        // ---------- Double Shot -------------
        if (dShotI == 0) {
            if (fabs(flywheelPid.target - dShotSpeed2) > 0.001) pidFlywheelInit(dShotSpeed2, 700);
            dShotI++;
        }
        if (dShotI == 1) {
            // load ball 1, wait for flywheel
            printf("dShot prep ball 1 ");
            intakeState = IntakeState::ALTERNATE;
            if (isPidFlywheelDone() && isBallIn()) {
                intakeRunning = false;
                pidIntakeInit(intakeShootTicks, 80);
                dShotI++;
            }
        } else if (dShotI == 2) {  // shoot ball 1
            printf("dShot shoot ball 1 ");
            if (pidIntake()) {
                intakeRunning = true;
                intakeState = IntakeState::FRONT;
                pidFlywheelInit(dShotSpeed1, 800);
                dShotI++;
            }
        } else if (dShotI == 3) {  // load ball 2, wait for flywheel
            printf("dShot prep ball 2 ");
            intakeState = IntakeState::ALTERNATE;
            if (isPidFlywheelDone() && isBallIn()) {
                intakeRunning = false;
                pidIntakeInit(intakeShootTicks, 80);
                dShotI++;
            }
        } else if (dShotI == 4) {  // shoot ball 2
            printf("dShot shoot ball 2 ");
            if (pidIntake()) dShotI = -1;
        }
        if (dShotI == -1 && prevDShotI != -1) {
            pidFlywheelInit(sShotSpeed, 700);
            intakeRunning = true;
            intakeState = IntakeState::FRONT;
        }
        prevDShotI = dShotI;
        pidFlywheel();
        // printf("{req %d actl %d}", getFlywheelVoltage(), mtr6.get_voltage());
        // drfb
        double drfbPos = getDrfb();
        /*if (curClicks[ctlrIdxR1] && (curClicks[ctlrIdxY] || curClicks[ctlrIdxA])) {
            if (!prevClicks[ctlrIdxR1]) drfbIMEBias -= 10;
        } else if (curClicks[ctlrIdxR2] && (curClicks[ctlrIdxY] || curClicks[ctlrIdxA])) {
            if (!prevClicks[ctlrIdxR2]) drfbIMEBias += 10;
        } else */
        if (curClicks[ctlrIdxR1]) {
            atDrfbSetp = false;
            drfbPidRunning = false;
            drfbFullRangePowerLimit = 12000;
            autoFlipI = -1;

            tDrfbOff = millis();
            setDrfb(12000);
        } else if (curClicks[ctlrIdxR2]) {
            atDrfbSetp = false;
            drfbPidRunning = false;
            drfbFullRangePowerLimit = 12000;
            autoFlipI = -1;

            tDrfbOff = millis();
            setDrfb(-12000);
        } else if (curClicks[ctlrIdxY]) {
            atDrfbSetp = true;
            drfbPidRunning = true;
            drfbFullRangePowerLimit = 12000;
            autoFlipI = -1;
            drfbPidBias = 0;

            drfbPid.target = drfbPos1;
            setDrfbParams(true);
        } else if (curClicks[ctlrIdxA]) {
            atDrfbSetp = true;
            drfbPidRunning = true;
            drfbFullRangePowerLimit = (getDrfb() > drfbPos2 + 50) ? 5000 : 12000;
            autoFlipI = -1;
            drfbPidBias = 0;

            drfbPid.target = drfbPos2;
            setDrfbParams(true);
        } else if (autoFlipI > -1) {
            if (autoFlipI == 0) {
                printf("auto flip step 0 ");
                if (getDrfb() < drfbMinClaw0) {
                    atDrfbSetp = false;
                    drfbPidRunning = true;
                    drfbFullRangePowerLimit = 12000;
                    setDrfbParams(true);
                    drfbPidBias = 5000;
                    autoFlipH = 0;
                    drfbPid.target = drfb18Max;
                    autoFlipI++;
                } else if (fabs(getDrfb() - drfbPos1) < 100) {
                    atDrfbSetp = false;
                    drfbPidRunning = true;
                    drfbFullRangePowerLimit = 12000;
                    setDrfbParams(true);
                    drfbPidBias = 5000;
                    autoFlipH = 1;
                    drfbPid.target = drfbPos1Plus;
                    autoFlipI++;
                } else if (fabs(getDrfb() - drfbPos2) < 100) {
                    atDrfbSetp = false;
                    drfbPidRunning = true;
                    drfbFullRangePowerLimit = 12000;
                    setDrfbParams(true);
                    drfbPidBias = 6000;
                    autoFlipH = 2;
                    drfbPid.target = drfbPos2Plus;
                    autoFlipI++;
                } else {
                    autoFlipI = -1;
                }
            } else if (autoFlipI == 1) {
                printf("auto flip step 1 ");
                if (getDrfb() > drfbPid.target - 200) {
                    clawFlipRequest = true;
                    autoFlipI++;
                }
            } else if (autoFlipI == 2) {
                printf("auto flip step 2 ");
                if (getDrfb() > drfbPid.target) drfbPidBias = 0;
                if (!clawFlipRequest && fabs(getClaw() - clawPid.target) < claw180 * (autoFlipH == 2 ? 0.3 : (autoFlipH == 1 ? 0.37 : 0.45))) {
                    drfbPidBias = 0;
                    if (autoFlipH == 0) {
                        drfbPid.target = drfbPos0;
                        drfbFullRangePowerLimit = 12000;
                    } else if (autoFlipH == 1) {
                        drfbPid.target = drfbPos1;
                        drfbFullRangePowerLimit = 12000;
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
                if (fabs(getDrfb() - drfbPid.target) < 100) autoFlipI = -1;
            }
        } else if (millis() - tDrfbOff > 130 && millis() - opcontrolT0 > 300) {
            if (!drfbPidRunning) {
                drfbPidBias = 0;
                drfbPidRunning = true;
                drfbPid.target = getDrfb();
                setDrfbParams(false);
            }
        } else if (!drfbPidRunning) {
            setDrfb(0);
        }
        if (drfbPidRunning) pidDrfb();

        // CLAW
        if (curClicks[ctlrIdxX] && !prevClicks[ctlrIdxX] && autoFlipI == -1) {
            if (atDrfbSetp && (fabs(getDrfb() - drfbPos1) < 100 || fabs(getDrfb() - drfbPos2) < 100) || getDrfb() < drfbMinClaw0) {
                // request an auto-flip
                autoFlipI = 0;
            } else {
                clawFlipRequest = true;
            }
        }
        if (clawFlipRequest && millis() - opcontrolT0 > 300) {
            // move the drfb to within an acceptable range
            if (getDrfb() > drfbMaxClaw0 && getDrfb() < drfbMinClaw1) {
                drfbPidRunning = true;
                drfbFullRangePowerLimit = 12000;
                drfbPidBias = 0;
                setDrfbParams(true);
                drfbPid.target = drfbMinClaw1 + 120;
            }
            // fullfill the request if the drfb is within an acceptable range
            if (getDrfb() > drfbMinClaw0 && getDrfb() < drfbMaxClaw0 || getDrfb() > drfbMinClaw1) {
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
            intakeRunning = true;
            intakeState = IntakeState::NONE;
        } else if (curClicks[ctlrIdxL2]) {
            if (dShotI == 4) dShotI = -1;
            intakeRunning = true;
            intakeState = IntakeState::FRONT;
        } else if (curClicks[ctlrIdxL1]) {
            if (dShotI == 4) dShotI = -1;
            intakeRunning = true;
            intakeState = IntakeState::BACK_SLOW;
        } else if (dShotI == -1) {
            intakeRunning = true;
            if (intakeState == IntakeState::BACK_SLOW) { intakeState = IntakeState::ALTERNATE; }
            if (isBallIn() && intakeState == IntakeState::ALTERNATE) intakeState = IntakeState::NONE;
            // prevent stall
            /*if (intakeState == IntakeState::FRONT && intakeSaver.isPwr(0.7) && !intakeSaver.isFaster(0.2)) { intakeState = IntakeState::ALTERNATE; }*/
        }
        if (intakeRunning) setIntake(intakeState);
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