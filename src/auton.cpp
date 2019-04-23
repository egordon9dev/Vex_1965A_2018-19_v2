#include "auton.hpp"
#include "Point.hpp"
#include "main.h"
#include "setup.hpp"

using pros::delay;
using std::cout;
using std::endl;
/*
fix this:   timeBetweenI should be set at each step

*/
/*
/*

 ##     ##    ###    #### ##    ##    ########     ###     ######  ##    ##
 ###   ###   ## ##    ##  ###   ##    ##     ##   ## ##   ##    ## ##   ##
 #### ####  ##   ##   ##  ####  ##    ##     ##  ##   ##  ##       ##  ##
 ## ### ## ##     ##  ##  ## ## ##    ########  ##     ## ##       #####
 ##     ## #########  ##  ##  ####    ##     ## ######### ##       ##  ##
 ##     ## ##     ##  ##  ##   ###    ##     ## ##     ## ##    ## ##   ##
 ##     ## ##     ## #### ##    ##    ########  ##     ##  ######  ##    ##

*/
void autonMainBack(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    int g = 0;
    double targetAngle = -PI / 2;
    const int driveT = 50;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL, t02 = BIL, t03 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false, intakeRunning = true;
    int prevITime = millis();
    int timeBetweenI = 7000;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;

    // tuning setpoints
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, sweepBeforeC2, ptBeforeC2, ptC2, ptBeyondC2, ptAfterC2, ptAfterC22, sweepAfterC2, sweepBalls, pivotBeforeShoot2, ptShoot2;
    double turnAfterC2;
    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    if (leftSide) {
        return;
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 38);
        ptBeforeShoot = Point(0, -0.5);
        ptShoot = ptBeforeShoot + polarToRect(6, -0.00212);  // near post
        // pivotBeforeC2 = Point(-20, 29);
        sweepBeforeC2 = 0.95 * Point(56, 27);
        ptBeforeC2 = Point(-14, 22);
        ptC2 = Point(-13, 26);
        ptBeyondC2 = Point(-13, 42);
        ptAfterC2 = Point(-13, 35);
        ptAfterC22 = Point(-13, 32);
        sweepAfterC2 = Point(-33, -10);
        turnAfterC2 = 0.88 * PI;
        sweepBalls = Point(-26, -37.62);
        // ptBalls = Point(-14, 44);
        pivotBeforeShoot2 = Point(-8, 33);
        ptShoot2 = pivotBeforeShoot2 + polarToRect(21, 0.594);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(2.97, 0.1, 500);
    pidDriveLineInit(pt0, ptBeforeC1, true, 0.15, 0);
    setDriveSlew(true);

    while (!ctlr.get_digital(DIGITAL_B)) {
        printf("%.2f ", (millis() - autonT0) / 1000.0);
        pros::lcd::print(8, "Time: %d ms", millis() - autonT0);
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 15000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) {
            printf("\ntimeBetweenI exceeded breaking....\n");
            break;
        }
        int j = 0;
        if (i == j++) {  // grab ball from under cap 1
            timeBetweenI = 2800;
            printf("drv to cap 1 ");
            clawPidRunning = true;
            clawPid.target = claw0;
            drfbPidRunning = true;
            drfbPid.target = 300;
            if (getDrfb() > drfbPosCloseIntake - 70) is = IntakeState::FRONT;
            bool driveDone = false;
            if (k == 0) {
                bool driveDone = pidDriveLine();
                if (driveDone) {
                    if (millis() - t0 < 500) {
                        pidDriveLineInit(pt0, ptBeforeC1, true, 0.15, 0);
                    } else if ((odometry.getPos() - ptBeforeC1).mag() < 0.5) {
                        k++;
                    }
                }
            } else if (k == 1) {
                setDL(-3001);
                setDR(-3001);
                if (odometry.getY() > ptC1.y - 0.5 || isBtmBallIn()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.15, driveT);
                t0 = BIL;
                t03 = millis();
                is = IntakeState::FRONT;
                i++;
            }
        } else if (i == j++) {  // drive to ptBeforeShoot
            timeBetweenI = 2700;
            printf("drive to ptBeforeShoot ");
            if (odometry.getY() < 10) {
                drfbPidRunning = false;
                if (getDrfb() > drfbPos0) t02 = millis();
                setDrfb(millis() - t02 < 500 ? -12000 : -1500);
                is = IntakeState::FRONT_HOLD;
            }
            if (pidDriveLine() && millis() - t03 > 500) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, true, 0.08, 100);
                k = 0;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot, shoot side post flags
            timeBetweenI = 4200;
            printf("drive to ptShoot ");
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0) t02 = millis();
            setDrfb(millis() - t02 < 500 ? -12000 : -1500);
            if (pidDriveLine()) {
                if (k == 0) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                } else if (k == 1) {
                    if (millis() - t03 > 450) {  // 300 minimum
                        pidSweepInit(sweepBeforeC2.x, sweepBeforeC2.y, 2.0, 0);
                        pidFlywheelInit(3.08, 0.1, 500);
                        k = 0;
                        i++;
                    }
                }
            } else {
                is = IntakeState::FRONT_HOLD;
            }
        } else if (i == j++) {  // drive to C2
            timeBetweenI = 3800;
            printf("drive to C2 ");
            is = IntakeState::NONE;
            drfbPidRunning = true;
            if (k == 0) {
                drfbPid.target = drfbPosAboveScrape;
                printf("sweep ");
                pidSweep();
                if (fabs(drivePid.target - drivePid.sensVal) < 6) {
                    pidDriveLineInit(odometry.getPos(), ptBeforeC2, false, 0.3, driveT);
                    k++;
                }
            } else if (k == 1) {
                printf("vision ");
                // pidDriveLine();
                if (odometry.getY() > ptC2.y) {
                    setDL(3700);
                    setDR(3700);
                } else {
                    // driving backwards means negative power
                    drivePid.target = ptC2.y + 0.4 - odometry.getY();
                    drivePid.sensVal = 0;
                    bool isRedCap = !leftSide;
                    double approxA = 4.71;
                    double mae = 0.25;
                    if (odometry.getA() < approxA - mae) {
                        setDL(0);
                        setDR(12000);
                    } else if (odometry.getA() > approxA + mae) {
                        setDL(12000);
                        setDR(0);
                    } else {
                        driveToCap(isRedCap, clamp(lround(drivePid.update()), -6000, 6000), 1.0);
                    }
                }
                if (fabs(ptBeyondC2.y - odometry.getY()) < 5) { drfbPid.target = drfbPosScrape; }
                if (fabs(ptBeyondC2.y - odometry.getY()) < 3) {
                    driveLim = 12000;
                    pidDriveLineInit(ptBeyondC2, ptAfterC2, true, 99999, 170);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            timeBetweenI = 4400;
            printf("scrape balls off C2 ");
            if (k == 0) {
                if (getDrfb() < drfbPosScrape + 80) {
                    printf("scrape ");
                    if (odometry.getY() < ptAfterC2.y + 0.5) drfbPidRunning = true;
                    drfbPid.target = drfb18Max;
                } else {
                    drfbPidRunning = true;
                    drfbPid.target = drfbPosScrape;
                }
                if (pidDriveLine()) {
                    // pidDriveLineInit(ptAfterC2, ptAfterC22, true, 0.1, 0);
                    t0 = millis();
                    g = 0;
                    k++;
                } else {
                    drfbPidRunning = false;
                    setDL(0);
                    setDR(0);
                    setDrfb(-12000);
                }
            } else if (k == 1) {
                printf(" sweep ");
                bool sweeping = false;
                if (millis() - t0 < 150) {
                    drfbPidRunning = false;
                    setDrfb(12000);
                    setDL(1000);
                    setDR(1000);
                } else {
                    drfbPidRunning = true;
                    drfbPid.target = drfb18Max;
                    if (g == 0) {
                        // pidDriveLine();
                        setDL(-6000);
                        setDR(-6000);
                        if (odometry.getY() < ptAfterC22.y) {
                            g++;
                            pidSweepInit(sweepAfterC2.x, sweepAfterC2.y, 2.0, 0);
                        }
                    } else {
                        sweeping = true;
                        pidSweep();
                    }
                }
                if (sweeping && fabs(drivePid.target - drivePid.sensVal) < 2) {
                    pidTurnInit(odometry.getA() + turnAfterC2, 100);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            timeBetweenI = 4000;
            printf("grab Balls ");
            drfbPid.target = drfbPosCloseIntake;
            if (k == 0) {  // Turn to face balls
                printf("Turn ");
                pidTurn();
                if (fabs(turnPid.target - turnPid.sensVal) < 0.2) {
                    // pidDriveLineInit(odometry.getPos(), ptBalls, true, 0.1, 0);
                    pidSweepInit(sweepBalls.x, sweepBalls.y, 1.4, 0);
                    k++;
                }
            } else if (k == 1) {  // drive to grab balls
                printf("Drive ");
                is = IntakeState::FRONT;
                pidSweep();
                if (fabs(drivePid.target - drivePid.sensVal) < 1) { k++; }
            } else if (k == 2) {
                printf("Drivemore ");
                setDL(-9000);
                setDR(-9000);
                if (/*odometry.getY() > ptBalls.y */ 1) {
                    pidDriveLineInit(odometry.getPos(), pivotBeforeShoot2, false, 0.15, driveT);
                    t0 = BIL;
                    i++;
                }
            }
        } else if (i == j++) {  // drive to pivotBeforeShoot2
            timeBetweenI = 2800;
            printf("drive twd pivotBeforeShoot2 ");
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 300) { is = IntakeState::FRONT_HOLD; }
            if (pidDriveLine()) {
                pidDriveLineInit(pivotBeforeShoot2, ptShoot2, true, 0.08, 100);
                t0 = BIL;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot2
            timeBetweenI = 3400;
            printf("drive to ptShoot2 ");
            if (getDrfb() > drfbPos0) t02 = millis();
            // bool intakeDone = isTopBallIn() && isBtmBallIn() || fabs(drivePid.target - drivePid.sensVal) < 14;
            is = IntakeState::FRONT_HOLD;
            drfbPidRunning = false;
            setDrfb(millis() - t02 > 500 ? -1500 : -12000);
            if (pidDriveLine()) {
                t0 = millis();
                i++;
                is = IntakeState::FRONT;
            }
        } else if (i == j++) {
            timeBetweenI = 1200;
            pidDriveLine();
            setDrfb(-1500);
            if (millis() - t0 > 600) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            timeBetweenI = 300;
            setDL(12000);
            setDR(12000);
            if (millis() - t0 > 150) { i++; }
        } else if (i == j++) {
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            stopMotors();
            break;
        }
        printPidValues();
        clawPid.sensVal = getClaw();
        setClaw(clawPid.update());
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}
void autonSupportBack(bool leftSide) {}
void autonFlagSide(bool leftSide) {}

void autonomous() {
    stopOpctlTask();
    odometry.reset();
    setDriveSlew(true);
    if (autoSel_nAuton == 0) {
        stopMotorsBlock();
    } else if (autoSel_nAuton == 1) {
        autonMainBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        autonSupportBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 3) {
        autonFlagSide(autoSel_leftSide);
    }
    stopMotorsBlock();
}
