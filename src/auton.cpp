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
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, sweepShoot, ptBeforeC2, ptC2, ptAfterC2, pivotBeforePost, ptPost;
    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    if (leftSide) {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 37.5);
        ptBeforeShoot = Point(0, -2);
        ptShoot = ptBeforeShoot + polarToRect(8, PI + 0.095);  // near post
        sweepShoot = Point(-7, -7);
        ptBeforeC2 = Point(13, 33);
        ptC2 = Point(19.5, 43);
        pivotBeforePost = Point(10, 10);
        ptPost = Point(10, -10);
        ptAfterC2 = Point(8, 20);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 37.5);
        ptBeforeShoot = Point(0, -2);
        ptShoot = ptBeforeShoot + polarToRect(8, -0.006);  // near post
        sweepShoot = Point(-7, -7);
        ptBeforeC2 = Point(-13, 33);
        ptC2 = Point(-18, 40);
        pivotBeforePost = Point(-10, 10);
        ptPost = Point(-18, -10);
        ptAfterC2 = Point(-8, 20);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(2.98, 0.1, 500);
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
                if (pidDriveLine() || (odometry.getPos() - ptBeforeC1).mag() < 0.5) k++;
            } else if (k == 1) {
                setDL(-4000);
                setDR(-4000);
                if (odometry.getY() > ptC1.y - 0.5 || isBtmBallIn()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.1, 0);
                t0 = BIL;
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
            if (pidDriveLine()) {
                pidFaceInit(ptShoot, true, 0);
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
            if (k == 0) {
                is = IntakeState::FRONT_HOLD;
                if (pidFace()) {
                    k++;
                    pidSweepInit(sweepShoot.x, sweepShoot.y, 1.0, 100);
                }
            } else if (k == 1) {
                if (pidSweep()) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                }
            } else {
                if (millis() - t03 > 410) {  // 300 minimum
                    pidFlywheelInit(0.0, 0.1, 500);
                    pidDriveLineInit(odometry.getPos(), ptC2, false, 0.2, 0);
                    t0 = millis();
                    i++;
                }
            }
        } else if (i == j++) {  // drive to C2
            timeBetweenI = 3800;
            printf("drive to C2 ");
            is = IntakeState::NONE;
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0) t0 = millis();
            setDrfb(millis() - t0 < 500 ? -12000 : -1500);
            if (pidDriveLine()) {
                driveLim = 12000;
                k = 0;
                pidDriveLineInit(odometry.getPos(), pivotBeforePost, true, 0.1, 0);
                i++;
            }
        } else if (i == j++) {
            timeBetweenI = 99999;
            if (getDrfb() > drfbPos0 + 100) {
                pidDriveLine();
            } else {
                setDL(0);
                setDR(0);
            }
            if (k == 0) {
                drfbPidRunning = true;
                drfbPid.target = drfbMinClaw0 + 100;
                if (getDrfb() > drfbMinClaw0) {
                    k++;
                    clawPidRunning = false;
                    t0 = millis();
                }
            } else if (k == 1) {
                setClaw(12000);
                if (millis() - t0 > 500) {
                    setClaw(0);
                    t0 = millis();
                    t02 = millis();
                    driveLim = 6500;
                    pidDriveLineInit(pivotBeforePost, ptPost, false, 0.1, 0);
                    i++;
                }
            }
        } else if (i == j++) {
            drfbPid.target = drfbPos0;
            setClaw(0);
            pidDriveLine();
            if (odometry.getY() < 0 && fabs(getDriveVel()) < 0.3) { i++; }
        } else if (i == j++) {
            setClaw(0);
            drfbPid.target = drfbPos1Plus;
            setDL(-9000);
            setDR(-9000);
            if (odometry.getY() > 5) {
                pidDriveLineInit(odometry.getPos(), ptPost, false, 0.1, 0);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            setClaw(0);
            setDL(9000);
            setDR(9000);
            if (millis() - t0 > 600 && fabs(getDriveVel()) < 0.3) { i++; }
        } else if (i == j++) {
            setClaw(0);
            setDL(0);
            setDR(0);
            setDrfb(-12000);
            drfbPidRunning = false;
            if (getDrfb() < drfbPos1 + 70) i++;
        } else if (i == j++) {
            setClaw(0);
            drfbPidRunning = true;
            drfbPid.target = drfbPos1;
            setDL(-9000);
            setDR(-9000);
            if (odometry.getY() > 6) i++;
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
        setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}
/*
  ######  ##     ## ########  ########   #######  ########  ########    ########     ###     ######  ##    ##
 ##    ## ##     ## ##     ## ##     ## ##     ## ##     ##    ##       ##     ##   ## ##   ##    ## ##   ##
 ##       ##     ## ##     ## ##     ## ##     ## ##     ##    ##       ##     ##  ##   ##  ##       ##  ##
  ######  ##     ## ########  ########  ##     ## ########     ##       ########  ##     ## ##       #####
       ## ##     ## ##        ##        ##     ## ##   ##      ##       ##     ## ######### ##       ##  ##
 ##    ## ##     ## ##        ##        ##     ## ##    ##     ##       ##     ## ##     ## ##    ## ##   ##
  ######   #######  ##        ##         #######  ##     ##    ##       ########  ##     ##  ######  ##    ##
*/
void autonSupportBack(bool leftSide) {
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
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, sweepShoot, ptBeforeC2, ptC2, ptAfterC2, pivotBeforePost, ptPost;
    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    // FIIIIIX THEEESE

    if (leftSide) {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 37.5);
        ptBeforeShoot = Point(0, 3);
        ptShoot = ptBeforeShoot + polarToRect(1, PI - 0.270);  // center post
        sweepShoot = Point(-15, -15);
        ptBeforeC2 = Point(13, 33);
        ptC2 = Point(19, 42);
        pivotBeforePost = Point(10, 10);
        ptPost = Point(18, -10);
        ptAfterC2 = Point(8, 20);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 37.5);
        ptBeforeShoot = Point(0, 3);
        ptShoot = ptBeforeShoot + polarToRect(1, 0.350);  // center post
        sweepShoot = Point(-15, -15);
        ptBeforeC2 = Point(-13, 33);
        ptC2 = Point(-19, 42);
        pivotBeforePost = Point(-10, 10);
        ptPost = Point(-18, -10);
        ptAfterC2 = Point(-8, 20);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(3.10, 0.1, 500);
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
                if (pidDriveLine() || (odometry.getPos() - ptBeforeC1).mag() < 0.5) k++;
            } else if (k == 1) {
                setDL(-4000);
                setDR(-4000);
                if (odometry.getY() > ptC1.y - 0.5 || isBtmBallIn()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.1, 0);
                t0 = BIL;
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
            if (pidDriveLine()) {
                pidFaceInit(ptShoot, true, 0);
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
            if (k == 0) {
                is = IntakeState::FRONT_HOLD;
                if (pidFace()) {
                    k++;
                    pidSweepInit(sweepShoot.x, sweepShoot.y, 1.0, 100);
                }
            } else if (k == 1) {
                if (pidSweep()) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                }
            } else {
                if (millis() - t03 > 410) {  // 300 minimum
                    pidFlywheelInit(0.0, 0.1, 500);
                    pidDriveLineInit(odometry.getPos(), ptC2, false, 0.2, 0);
                    t0 = millis();
                    i++;
                }
            }
        } else if (i == j++) {  // drive to C2
            timeBetweenI = 3800;
            printf("drive to C2 ");
            is = IntakeState::NONE;
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0) t0 = millis();
            setDrfb(millis() - t0 < 500 ? -12000 : -1500);
            if (pidDriveLine()) {
                driveLim = 12000;
                k = 0;
                pidDriveLineInit(odometry.getPos(), pivotBeforePost, true, 0.1, 0);
                i++;
            }
        } else if (i == j++) {
            timeBetweenI = 99999;
            if (getDrfb() > drfbPos0 + 100) {
                pidDriveLine();
            } else {
                setDL(0);
                setDR(0);
            }
            if (k == 0) {
                drfbPidRunning = true;
                drfbPid.target = drfbMinClaw0 + 100;
                if (getDrfb() > drfbMinClaw0) {
                    k++;
                    clawPidRunning = false;
                    t0 = millis();
                }
            } else if (k == 1) {
                setClaw(12000);
                if (millis() - t0 > 500) {
                    setClaw(0);
                    t0 = millis();
                    t02 = millis();
                    driveLim = 6500;
                    pidDriveLineInit(pivotBeforePost, ptPost, false, 0.1, 0);
                    i++;
                }
            }
        } else if (i == j++) {
            setClaw(0);
            pidDriveLine();
            if (odometry.getY() < 0 && fabs(getDriveVel()) < 0.3) { i++; }
        } else if (i == j++) {
            setClaw(0);
            drfbPid.target = drfbPos1Plus;
            setDL(-9000);
            setDR(-9000);
            if (odometry.getY() > 0) {
                pidDriveLineInit(odometry.getPos(), ptPost, false, 0.1, 0);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            setClaw(0);
            setDL(9000);
            setDR(9000);
            if (millis() - t0 > 600 && fabs(getDriveVel()) < 0.3) { i++; }
        } else if (i == j++) {
            setClaw(0);
            setDL(0);
            setDR(0);
            setDrfb(-12000);
            drfbPidRunning = false;
            if (getDrfb() < drfbPos1 + 70) i++;
        } else if (i == j++) {
            setClaw(0);
            drfbPidRunning = true;
            drfbPid.target = drfbPos1;
            setDL(-9000);
            setDR(-9000);
            if (odometry.getY() > 6) i++;
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
        setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}
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
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, sweepShoot, ptBeforeC2, ptC2, ptAfterC2, pivotBeforePost, ptPost;
    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    if (leftSide) {
        ptBeforeC1 = Point(0, 34);
        ptC1 = Point(0, 36.5);
        ptBeforeShoot = Point(5, 25);
        ptShoot = ptBeforeShoot + polarToRect(1, PI - 0.64 + 0.05);  // far post
        sweepShoot = Point(-5, -5);
        ptBeforeC2 = Point(13, 33);
        ptC2 = Point(19.5, 43);
        pivotBeforePost = Point(10, 10);
        ptPost = Point(10, -10);
        ptAfterC2 = Point(8, 20);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeC1 = Point(0, 33);
        ptC1 = Point(0, 35.5);
        ptBeforeShoot = Point(-1, 31);
        ptShoot = ptBeforeShoot + polarToRect(1, 0.350);  // far post
        sweepShoot = Point(-4, -4);
        ptBeforeC2 = Point(-13, 33);
        ptC2 = Point(-19, 42);
        pivotBeforePost = Point(-10, 10);
        ptPost = Point(-18, -10);
        ptAfterC2 = Point(-8, 20);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(3.13, 0.1, 500);
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
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.15, 100);
                t0 = BIL;
                is = IntakeState::FRONT;
                i++;
            }
        } else if (i == j++) {  // drive to ptBeforeShoot
            printf("drive to ptBeforeShoot ");
            if (odometry.getY() < 10) {
                drfbPidRunning = false;
                if (getDrfb() > drfbPos0) t02 = millis();
                setDrfb(millis() - t02 < 500 ? -12000 : -1500);
                is = IntakeState::FRONT_HOLD;
            }
            if (pidDriveLine()) {
                pidFaceInit(ptShoot, true, 100);
                k = 0;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot, shoot side post flags
            printf("drive to ptShoot ");
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0) t02 = millis();
            setDrfb(millis() - t02 < 500 ? -12000 : -1500);
            if (k == 0) {
                is = IntakeState::FRONT_HOLD;
                if (pidFace()) {
                    k++;
                    t0 = millis();
                    pidSweepInit(sweepShoot.x, sweepShoot.y, 1.0, 0);
                }
            } else if (k == 1) {
                if (pidSweep() && millis() - t0 > 50) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                }
            } else {
                if (millis() - t03 > 1000) {  // 300 minimum
                    pidFlywheelInit(0.0, 0.1, 500);
                    pidDriveLineInit(odometry.getPos(), ptC2, false, 0.2, 0);
                    t0 = millis();
                    i++;
                }
            }
        } else if (i == j++) {
            setDL(12000);
            setDR(12000);
            if (millis() - t0 > 300) i++;
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
        setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}
void autonFlagSide(bool leftSide) {}

void autonomous() {
    odometry.reset();
    setDriveSlew(true);
    if (autoSel_nAuton == 0) {
        stopMotorsBlock();
    } else if (autoSel_nAuton == 1) {
        autonMainBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        autonSupportBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 3) {
        autonSupCrossBack(autoSel_leftSide);
    } else if (autoSel_nAuton == 4) {
        autonFlagSide(autoSel_leftSide);
    }
    stopMotorsBlock();
}
