#include "Point.hpp"
#include "auton.hpp"
#include "main.h"
#include "setup.hpp"

using pros::delay;

/*
  ######  ##     ## ########   ######  ########   #######   ######   ######  ########     ###     ######  ##    ##
 ##    ## ##     ## ##     ## ##    ## ##     ## ##     ## ##    ## ##    ## ##     ##   ## ##   ##    ## ##   ##
 ##       ##     ## ##     ## ##       ##     ## ##     ## ##       ##       ##     ##  ##   ##  ##       ##  ##
  ######  ##     ## ########  ##       ########  ##     ##  ######   ######  ########  ##     ## ##       #####
       ## ##     ## ##        ##       ##   ##   ##     ##       ##       ## ##     ## ######### ##       ##  ##
 ##    ## ##     ## ##        ##    ## ##    ##  ##     ## ##    ## ##    ## ##     ## ##     ## ##    ## ##   ##
  ######   #######  ##         ######  ##     ##  #######   ######   ######  ########  ##     ##  ######  ##    ##
*/
void autonSupCrossBack(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    int g = 0;
    double targetAngle = -PI / 2;
    const int driveT = 0;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL, t02 = BIL, t03 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = true, intakeRunning = true;
    int prevITime = millis();
    int timeBetweenI = 7000;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;
    setDrfbParams(true);

    // tuning setpoints
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptBeforeShoot2, ptShoot, sweepShoot;

    /*
     ##       ######## ######## ########
     ##       ##       ##          ##
     ##       ##       ##          ##
     ##       ######   ######      ##
     ##       ##       ##          ##
     ##       ##       ##          ##
     ######## ######## ##          ##
    */
    //
    if (leftSide) {  // 0 / 10
        ptBeforeC1 = Point(0, 33);
        ptC1 = Point(0, 35.5);
        ptBeforeShoot = Point(1, 31);
        ptBeforeShoot2 = ptBeforeShoot + polarToRect(-8, PI - 0.55 + 0.05);  // far post
        ptShoot = ptBeforeShoot2 + polarToRect(8, PI - 0.55 + 0.05);         // far post
        sweepShoot = Point(0, 0);
    }

    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    /*
     ########  ####  ######   ##     ## ########
     ##     ##  ##  ##    ##  ##     ##    ##
     ##     ##  ##  ##        ##     ##    ##
     ########   ##  ##   #### #########    ##
     ##   ##    ##  ##    ##  ##     ##    ##
     ##    ##   ##  ##    ##  ##     ##    ##
     ##     ## ####  ######   ##     ##    ##
    */
    //
    else {  // 6 / 10
        ptBeforeC1 = Point(0, 32.5);
        ptC1 = Point(0, 35);
        ptBeforeShoot = Point(-1, 31);
        ptBeforeShoot2 = ptBeforeShoot + polarToRect(-8, 0.5625);  // far post
        ptShoot = ptBeforeShoot2 + polarToRect(8, 0.562);          // far post
        sweepShoot = Point(0, 0);
    }
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(3.04, 0.1, 500);
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
            timeBetweenI = 999999;
            printf("drv to cap 1 ");
            clawPidRunning = true;
            clawPid.target = claw0;
            drfbPidRunning = true;
            drfbPid.target = 300;
            if (getDrfb() > drfbPosCloseIntake - 70) is = IntakeState::FRONT;
            bool driveDone = false;
            if (k == 0) {
                if (pidDriveLine() || (odometry.getPos() - ptBeforeC1).mag() < 0.5) k++;
            } else if (k == 1) {
                setDL(-3000);
                setDR(-3000);
                if (odometry.getY() > ptC1.y - 0.5 || isBtmBallIn()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.15, 500);
                t0 = BIL;
                t02 = BIL;
                is = IntakeState::FRONT;
                i++;
            }
        } else if (i == j++) {  // drive to ptBeforeShoot
            printf("drive to ptBeforeShoot ");
            if (isBtmBallIn() && isTopBallIn() && t02 > millis()) t02 = millis();
            if (millis() - t02 > 900) is = IntakeState::FRONT_HOLD;
            if (pidDriveLine()) {
                pidFaceInit(ptShoot, true, 100);
                k = 0;
                t03 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot, shoot side post flags
            printf("drive to ptShoot ");
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0) t03 = millis();
            setDrfb(millis() - t03 < 500 ? -12000 : -1500);
            if (k == 0) {
                is = IntakeState::FRONT_HOLD;
                if (pidFace()) {
                    k++;
                    pidDriveLineInit(ptBeforeShoot, ptBeforeShoot2, false, 0.1, 100);
                    t0 = millis();
                }
            } else if (k == 1) {
                pidDriveLine();
                if (millis() - t0 > 1000) {
                    pidDriveLineInit(ptBeforeShoot2, ptShoot, true, 0.1, 500);
                    t0 = millis();
                    g = 0;
                    k++;
                }
            } else if (k == 2) {
                if (g == 0) {
                    if (pidDriveLine()) g++;
                } else {
                    setDL(0);
                    setDR(0);
                }
                if (millis() - t0 > 7000) {
                    // intakeRunning = false;
                    // pidIntakeInit(420, 0);
                    is = IntakeState::FRONT;
                    t0 = millis();
                    k++;
                }
            } else if (k == 3) {  // 140ms min
                printf("11111111111 ");
                if (millis() - t0 > 200) {  // 300 minimum
                    is = IntakeState::BACK_HOLD;
                    pidFlywheelInit(2.72, 0.1, 500);
                    t0 = BIL;
                    k++;
                }
            } else if (k == 4) {
                pidIntake();
                if (fabs(flywheelPid.target - flywheelPid.sensVal) < 0.1 && t0 > millis()) t0 = millis();
                if (millis() - t0 > 1000) {
                    intakeRunning = true;
                    t0 = millis();
                    is = IntakeState::FRONT;
                    k++;
                }
            } else if (k == 5) {
                printf("22222222222 ");
                if (millis() - t0 > 1000 && !isTopBallIn()) { i++; }
            }
        } else {
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            stopMotors();
            break;
        }
        printPidValues();
        clawPid.sensVal = getClaw();
        if (clawPidRunning) setClaw(clawPid.update());
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        if (intakeRunning) setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}