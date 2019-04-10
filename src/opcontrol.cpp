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
    if (0) {
        while (1) {
            printf("%d %d\n", (int)(getDL() + 0.5), (int)(getDR() + 0.5));
            delay(10);
        }
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
        //	while(1) {
        //	printf("%.2f\n", getDrfb());
        //}
        auton5(true);
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
    int driveDir = 1;
    bool clawFlipRequest = false;
    setDrfbParams(false);
    driveLim = 12000;
    double atDrfbSetp = false;
    int autoFlipI = -1;

    int autoFlipH;
    int prevCtlrUpdateT = 0;
    bool prevIsBallIn = false;
    bool intakeToggle = false;
    bool fwPidRunning = false;
    int fwPwr = 7000;
    int prevNotHoldingT = -9999;
    // int iti = 0;    // iti = Intake Tracker Index
    // int itt = BIL;  // itt = Intake Tracker Time
    while (true) {
        pros::lcd::print(0, "x %f", odometry.getX());
        pros::lcd::print(1, "y %f", odometry.getY());
        pros::lcd::print(2, "a %f", odometry.getA());
        pros::lcd::print(3, "L %f", getDL());
        pros::lcd::print(4, "R %f", getDR());
        pros::lcd::print(5, "S %f", getDS());
        pros::lcd::print(7, "drfb%d: %.2f", getDrfbVoltage(), getDrfb());

        printf("t: %d ", millis());
        printPidValues();
        bool** allClicks = getAllClicks();
        bool prevClicks[12], curClicks[12], dblClicks[12];
        for (int i = 0; i < 12; i++) {
            prevClicks[i] = allClicks[0][i];
            curClicks[i] = allClicks[1][i];
            dblClicks[i] = allClicks[2][i];
        }

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) { driveDir *= -1; }
        // DRIVE
        driveLim = (getDrfb() > 0.5 * (drfb18Max + drfbPos1)) ? 8500 : 12000;
        opctlDrive(driveDir);
        // FLYWHEEL
        if (dblClicks[ctlrIdxDown]) {
            fwPidRunning = true;
            pidFlywheelInit(0, 0.1, 999);
        } else if (curClicks[ctlrIdxDown] && !prevClicks[ctlrIdxDown]) {
            fwPidRunning = false;
            fwPwr = 7000;
        } else if (curClicks[ctlrIdxUp]) {
            fwPidRunning = true;
            pidFlywheelInit(sShotSpeed, 0.1, 700);
        }

        if (fwPidRunning) {
            pidFlywheel();
        } else {
            setFlywheel(fwPwr);
        }
        // drfb
        double drfbPos = getDrfb();
        /* ------ Drfb Controls ------
        **       R1: Manual Up
        **       R2: Manual Down
        **       Y: Low Post
        **       A: High Post
        */
        if (curClicks[ctlrIdxR1] && getDrfb() < drfbPos2Plus) {
            atDrfbSetp = false;
            drfbPidRunning = false;
            drfbFullRangePowerLimit = 12000;
            autoFlipI = -1;
            tDrfbOff = millis();
            setDrfb(12000);
        } else if (curClicks[ctlrIdxR2] && getDrfb() >= drfbPosCloseIntake) {
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
        } else if (curClicks[ctlrIdxL2]) {
            atDrfbSetp = false;
            drfbPidRunning = true;
            drfbFullRangePowerLimit = 12000;
            autoFlipI = -1;
            drfbPidBias = 0;
            drfbPid.target = drfbPosCloseIntake;
            setDrfbParams(true);
        } else if (autoFlipI > -1) {
            if (autoFlipI == 0) {
                printf("auto flip step 0 ");
                if (getDrfb() < drfbMinClaw0) {
                    autoFlipH = 0;
                } else if (fabs(getDrfb() - drfbPos1) < 100) {
                    autoFlipH = 1;
                } else if (fabs(getDrfb() - drfbPos2) < 100) {
                    autoFlipH = 2;
                } else {
                    autoFlipH = -1;
                }
                if (autoFlipH >= 0) {
                    atDrfbSetp = false;
                    drfbPidRunning = true;
                    drfbFullRangePowerLimit = 12000;
                    setDrfbParams(true);
                    if (autoFlipH == 0) {
                        drfbPidBias = 5000;
                        drfbPid.target = drfb18Max;
                    } else if (autoFlipH == 1) {
                        drfbPidBias = 5000;
                        drfbPid.target = drfbPos1Plus;
                    } else {  // autoFlipH == 2
                        drfbPidBias = 6000;
                        drfbPid.target = drfbPos2Plus;
                    }
                    autoFlipI++;
                } else {
                    autoFlipI = -1;
                }
            } else if (autoFlipI == 1) {
                printf("auto flip step 1 ");
                if (getDrfb() > drfbPid.target - 100) {
                    clawFlipRequest = true;
                    autoFlipI++;
                }
            } else if (autoFlipI == 2) {
                printf("auto flip step 2 ");
                if (getDrfb() > drfbPid.target) drfbPidBias = 0;
                if (!clawFlipRequest && fabs(getClaw() - clawPid.target) < claw180 * (autoFlipH == 2 ? 0.3 : autoFlipH == 1 ? 0.37 : 0.45)) {
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
        } else if ((millis() - tDrfbOff > 130 && millis() - opcontrolT0 > 300) || getDrfb() < (drfbPos0 + drfbPosCloseIntake) * 0.5) {
            if (!drfbPidRunning) {
                drfbPidBias = 0;
                drfbPidRunning = true;
                drfbPid.target = getDrfb();
                setDrfbParams(false);
            }
        } else if (!drfbPidRunning) {
            printf("idling");
            setDrfb(0);
        }
        bool isDrfbHolding = false;
        if (drfbPidRunning) {
            if (drfbPid.target < drfbPosCloseIntake) {
                setDrfbDull(getDrfb() < drfbPos0 || millis() - prevNotHoldingT > 400 ? drfbHoldPwr : -12000);
                isDrfbHolding = true;
            } else {
                pidDrfb();
            }
        }
        if (!isDrfbHolding) prevNotHoldingT = millis();
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

        // -----------  Intake  ------------
        if (isTopBallIn() && isBtmBallIn() && !curClicks[ctlrIdxL1]) intakeToggle = false;
        if (curClicks[ctlrIdxL1] && !prevClicks[ctlrIdxL1]) { intakeToggle = !intakeToggle; }
        if (intakeToggle) {
            intakeState = IntakeState::FRONT;
        } else {
            intakeState = isTopBallIn() ? IntakeState::FRONT_HOLD : IntakeState::NONE;
        }
        setIntake(intakeState);
        if (millis() - prevCtlrUpdateT > 150) {
            bool curIsBallIn = isBtmBallIn();
            if (curIsBallIn && !prevIsBallIn) {
                ctlr.rumble(" .");
            } else {
                ctlr.print(2, 0, "Balls: %s, %s", isBtmBallIn() ? "BTM" : "---", isTopBallIn() ? "TOP" : "---");
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