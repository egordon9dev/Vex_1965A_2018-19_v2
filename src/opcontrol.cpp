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
bool robotInit = true;
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
        // while (1) {
        // printf("top %d    btm %d\n", getBallSensTop(), getBallSensBtm());
        // delay(10);
        // }

        printf("starting....\n");
        pros::Vision::print_signature(vision->get_signature(1));
        printf("\n");
        while (1) {
            pros::vision_object_s_t objs[1];
            int nObjs = vision->read_by_sig(0, 1, 1, objs);
            double x = objs[0].left_coord, y = objs[0].top_coord, w = objs[0].width, h = objs[0].height;
            x += w / 2;
            y += h / 2;
            turnPid.target = 0;
            turnPid.sensVal = (x - 175) * 0.0045;
            int out = clamp(lround(turnPid.update()), -12000, 12000);
            // printf("%.1f %.1f\n", x, y);
            delay(10);
        }

        // setDriveSlew(true);
        // while (!ctlr.get_digital(DIGITAL_A)) delay(10);
        // pidSweepInit(43.2, 24, 2.0, 9999);
        // while (!ctlr.get_digital(DIGITAL_B)) {
        //     printPidValues();
        //     pidSweep();
        //     delay(10);
        // }
        // while (1) {
        //     printf("%d %d %d %.2f %.2f %.2f\n", (int)getDL(), (int)getDR(), (int)getDS(), odometry.getX(), odometry.getY(), odometry.getA());
        //     delay(20);
        // }
        // for (int i = 0; i < 3; ++i) {
        //     int wait = 400;
        //     pidTurnInit(-PI / 2 + 1.5, wait);
        //     while (1) {
        //         setDrfb(-1500);
        //         if (pidTurn()) break;
        //         printDrivePidValues();
        //         delay(10);
        //     }
        //     pidTurnInit(-PI / 2, wait);
        //     while (1) {
        //         setDrfb(-1500);
        //         if (pidTurn()) break;
        //         printDrivePidValues();
        //         delay(10);
        //     }
        // }
        // stopMotorsBlock();
        // stopMotorsBlock();
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
        autonMainBack(false);
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
    IntakeState intakeState = IntakeState::FRONT_HOLD, isBeforeBack = IntakeState::NONE;
    int driveDir = 1;
    bool clawFlipRequest = false;
    setDrfbParams(false);
    driveLim = 12000;
    double atDrfbSetp = false;
    int autoFlipI = -1;

    int autoFlipH;
    int prevCtlrUpdateT = 0;
    bool prevIsBallIn = false;
    bool fwPidRunning = false;
    int fwPwr = 7000;
    int prevNotHoldingT = -9999;

    bool oneShotReq = false, prevOneShotReq = false, intakeRunning = true;
    int prevL1T = -9999;
    // int iti = 0;    // iti = Intake Tracker Index
    // int itt = BIL;  // itt = Intake Tracker Time
    while (true) {
        pros::lcd::print(0, "%1.2f %1.2f %1.2f", odometry.getX(), odometry.getY(), odometry.getA());
        pros::lcd::print(1, "L%3d R%3d S%3d", (int)lround(getDL()), (int)lround(getDR()), (int)lround(getDS()));
        pros::lcd::print(2, "L%5d R%5d", getDLVoltage(), getDRVoltage());
        pros::lcd::print(4, "fw %1.3f/%1.3f", flywheelPid.sensVal, flywheelPid.target);
        pros::lcd::print(3, "%d", getFlywheelMeasuredVoltage());
        pros::lcd::print(6, "%d %5d e%1.3f", fwPidRunning ? 1 : 0, getFlywheelVoltage(), fabs(flywheelPid.sensVal - flywheelPid.target));
        pros::lcd::print(7, "drfb(%d)%2d %d", drfbPidRunning, getDrfbVoltage() / 1000, lround(getDrfb()));

        printf("t: %d ", millis());
        printPidValues();
        bool** allClicks = getAllClicks();
        bool prevClicks[12], curClicks[12], dblClicks[12];
        for (int i = 0; i < 12; i++) {
            prevClicks[i] = allClicks[0][i];
            curClicks[i] = allClicks[1][i];
            dblClicks[i] = allClicks[2][i];
        }

        if (curClicks[ctlrIdxB] && !prevClicks[ctlrIdxB]) {
            if (driveDir == -1) intakeState = IntakeState::NONE;
            driveDir *= -1;
        }
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
        } else if ((millis() - tDrfbOff > 130 && millis() - opcontrolT0 > 300) || getDrfb() < drfbPosCloseIntake) {
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
            // protect against lift sag locking us out of moving the lift
            if (getDrfb() < drfbPosCloseIntake && drfbPid.target > drfbPosCloseIntake + 0.001 && drfbPid.target < drfbPosCloseIntake + 200) { drfbPid.target = drfbPos0; }
            // hold lift down
            if (drfbPid.target < drfbPosCloseIntake) {
                setDrfbDull(getDrfb() < drfbPos0 || millis() - prevNotHoldingT > 400 ? drfbHoldPwr : -12000);
                isDrfbHolding = true;
            }
            // pid lift
            else {
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
        if (curClicks[ctlrIdxL1]) prevL1T = millis();
        // auto lift drfb
        if (!isTopBallIn() && !isBtmBallIn() && !oneShotReq && autoFlipI == -1 && driveDir == -1) {
            if (!curClicks[ctlrIdxR2] && drfbPidRunning && drfbPid.target < drfbPosCloseIntake) {
                drfbFullRangePowerLimit = 12000;
                drfbPidBias = 0;
                drfbPid.target = drfbPosCloseIntake;
                setDrfbParams(true);
            }
        }
        // auto-off
        if (isTopBallIn() && isBtmBallIn() && millis() - prevL1T > 800) {
            if (intakeState != IntakeState::BACK) intakeState = IntakeState::FRONT_HOLD;
        }
        // toggle
        if (dblClicks[ctlrIdxL1]) {
            intakeState = IntakeState::FRONT_HOLD;
        } else if (curClicks[ctlrIdxL1]) {
            // /if (intakeState == IntakeState::FRONT_HOLD || intakeState == IntakeState::BACK) {
            intakeState = IntakeState::FRONT;
            //} else if (intakeState == IntakeState::FRONT) {
            //    intakeState = IntakeState::FRONT_HOLD;
            //}
        }
        // reverse
        if (curClicks[ctlrIdxLeft] && !prevClicks[ctlrIdxLeft]) {
            isBeforeBack = intakeState;
            intakeState = IntakeState::BACK;
        }
        if (!curClicks[ctlrIdxLeft] && prevClicks[ctlrIdxLeft]) { intakeState = isBeforeBack; }
        // one shot
        if (curClicks[ctlrIdxRight] && !prevClicks[ctlrIdxRight]) oneShotReq = true;
        intakeRunning = !oneShotReq;
        if (oneShotReq && !prevOneShotReq) {
            intakeRunning = false;
            pidIntakeInit(isBtmBallIn() ? intakeOneShotTicksTop : intakeOneShotTicks, 100);
        }
        prevOneShotReq = oneShotReq;
        if (intakeRunning) {
            setIntake(intakeState);
        } else {
            if (pidIntake()) oneShotReq = false;
        }
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
    // delete perpindicularWheelEnc;
    delete gyro;
    delete vision;
    delete DLEnc;
    delete DREnc;
}

/*
#define SIG_1 1

pros::Vision vision_sensor(6);
void opcontrol() {
printf("starting....");
while (true) {
   int n = vision_sensor.get_object_count();
   pros::lcd::print(0, "%d ", n);

   if (n > 0) {
       pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, SIG_1);
       pros::lcd::print(1, "x: %3d y: %3d", obj.x_middle_coord, obj.y_middle_coord);
       pros::lcd::print(2, "w: %3d h: %3d", obj.width, obj.height);
   } else {
       pros::lcd::print(1, "no objects");
       pros::lcd::print(2, "          ");
   }

   pros::delay(20);
}
}

*/