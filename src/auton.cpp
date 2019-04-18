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
 ##    ## ########    ###    ########     ##     ## #### ########
 ###   ## ##         ## ##   ##     ##    ###   ###  ##  ##     ##
 ####  ## ##        ##   ##  ##     ##    #### ####  ##  ##     ##
 ## ## ## ######   ##     ## ########     ## ### ##  ##  ##     ##
 ##  #### ##       ######### ##   ##      ##     ##  ##  ##     ##
 ##   ### ##       ##     ## ##    ##     ##     ##  ##  ##     ##
 ##    ## ######## ##     ## ##     ##    ##     ## #### ########
*/
void autonNearMid(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 0;
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
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, pivotBeforeC2, ptC2, ptBeyondC2, ptAfterC2, ptAfterC22, ptBalls, pivotBeforeShoot2, ptShoot2, ptPost2;
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
        ptBeforeC1 = Point(0, 30);
        ptC1 = Point(0, 40);
        ptBeforeShoot = Point(-2, -2.3);
        ptShoot = Point(8, -2.15);  // near post
        pivotBeforeC2 = Point(-20, 29);
        ptC2 = Point(-20, 34);
        ptBeyondC2 = Point(-20, 38);
        ptAfterC2 = Point(-20, 31);
        ptAfterC22 = Point(-20, 10);
        turnAfterC2 = -PI;
        ptBalls = Point(-18, 35);
        pivotBeforeShoot2 = Point(-4, 30);
        ptShoot2 = pivotBeforeShoot2 + polarToRect(10, 0.405);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(sShotSpeed, 0.1, 500);
    pidDriveLineInit(pt0, ptBeforeC1, true, 0.1, 0);
    setDriveSlew(true);

    while (!ctlr.get_digital(DIGITAL_B)) {
        printf("%.2f ", (millis() - autonT0) / 1000.0);
        pros::lcd::print(8, "Time: %d ms", millis() - autonT0);
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 15000000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) {
            printf("\ntimeBetweenI exceeded breaking....\n");
            break;
        }
        int j = 0;
        if (i == j++) {  // grab ball from under cap 1
            printf("drv to cap 1 ");
            clawPidRunning = true;
            clawPid.target = 0;
            drfbPidRunning = true;
            drfbPid.target = 300;
            if (getDrfb() > drfbPosCloseIntake - 70) is = IntakeState::FRONT;
            bool driveDone = false;
            if (k == 0) {
                if (pidDriveLine() || (odometry.getPos() - ptBeforeC1).mag() < 2) { k++; }
            } else if (k == 1) {
                setDL(-6000);
                setDR(-6000);
                if (odometry.getY() > ptC1.y - 0.5 || isBtmBallIn()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.1, driveT);
                t0 = BIL;
                i++;
            }
        } else if (i == j++) {  // drive to ptBeforeShoot
            printf("drive to ptBeforeShoot ");
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 700) is = IntakeState::FRONT_HOLD;
            if (pidDriveLine() || (odometry.getPos() - ptBeforeShoot).mag() < 1.5) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, true, 0.08, driveT);
                k = 0;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot, shoot side post flags
            printf("drive to ptShoot ");
            bool driveDone = pidDriveLine();
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0 + 50) t02 = millis();
            setDrfb(millis() - t02 < 500 ? -12000 : -1500);
            if (driveDone) {
                if (k == 0) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                } else if (k == 1) {
                    if (millis() - t03 > 450) {  // 300 minimum
                        pidDriveLineInit(ptShoot, pivotBeforeC2, false, 0.1, 1);
                        pidFlywheelInit(3.0, 0.1, 500);
                        k = 0;
                        i++;
                    }
                }
            } else {
                if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
                if (millis() - t0 > 700) { is = IntakeState::FRONT_HOLD; }
            }
        } else if (i == j++) {  // drive to C2
            printf("drive to C2 ");
            is = IntakeState::NONE;
            drfbPidRunning = true;
            if (k == 0) {
                drfbPid.target = drfb18Max - 30;
                if (pidDriveLine() || (odometry.getPos() - pivotBeforeC2).mag() < 1.5) {
                    pidDriveLineInit(pivotBeforeC2, ptC2, false, 0.1, driveT);
                    k++;
                }
            } else if (k == 1) {
                drfbPid.target = 500;
                if (pidDriveLine() || ((odometry.getPos() - ptC2).mag() < 3 && (ptC2 - pivotBeforeC2).angleBetween(polarToRect(1, odometry.getA() + PI)) < 0.1)) { k++; }
            } else if (k == 2) {
                setDL(5000);
                setDR(5000);
                if (fabs(ptBeyondC2.y - odometry.getY()) < 3) {
                    drfbPidRunning = false;
                    setDrfb(getDrfb() < drfbPosScrape + 80 ? -1500 : -8000);
                } else {
                    drfbPidRunning = true;
                    drfbPid.target = 500;
                }
                if (odometry.getY() > ptBeyondC2.y - 0.5) {
                    pidDriveLineInit(ptBeyondC2, ptAfterC2, true, 0.1, driveT);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("scrape balls off C2 ");
            if (k == 0) {
                if (getDrfb() < drfbPosScrape + 80) {
                    drfbPidRunning = false;
                    setDrfb(-1500);
                    if (pidDriveLine()) {
                        pidDriveLineInit(ptAfterC2, ptAfterC22, true, 0.1, 0);
                        k++;
                    }
                } else {
                    drfbPidRunning = false;
                    setDL(0);
                    setDR(0);
                    setDrfb(-12000);
                }
            } else if (k == 1) {
                drfbPidRunning = true;
                drfbPidBias = 1000;
                drfbPid.target = drfbPosCloseIntake;
                if (pidDriveLine() || (odometry.getPos() - ptAfterC22).mag() < 2) {
                    pidTurnInit(odometry.getA() + turnAfterC2, 0);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("grab Balls ");
            if (k == 0) {  // Turn to face balls
                printf("Turn ");
                if (pidTurn()) {
                    pidDriveLineInit(odometry.getPos(), ptBalls, true, 0.1, 0);
                    k++;
                }
            } else if (k == 1) {  // drive to grab balls
                printf("Drive ");
                is = IntakeState::FRONT;
                if (pidDriveLine()) {
                    pidDriveLineInit(ptBalls, pivotBeforeShoot2, true, 0.1, driveT);
                    t0 = BIL;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive twd pivotBeforeShoot2 ");
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 300) { is = IntakeState::FRONT_HOLD; }
            if (pidDriveLine() || (odometry.getPos() - pivotBeforeShoot2).mag() < 2) {
                pidDriveLineInit(pivotBeforeShoot2, ptShoot2, true, 0.1, driveT);
                t0 = BIL;
                t02 = millis();
                t03 = BIL;
                i++;
            }
        } else if (i == j++) {
            printf("drive to ptShoot2 ");
            if (getDrfb() > drfbPos0 + 50) t02 = millis();
            bool intakeDone = isTopBallIn() && isBtmBallIn() || millis() - t03 > 500;
            if (intakeDone) {
                drfbPidRunning = false;
                setDrfb(millis() - t02 > 500 ? -1500 : -12000);
            } else {
                drfbPidRunning = true;
                drfbPid.target = drfbPosCloseIntake;
            }
            if (pidDriveLine()) {
                if (t03 > millis()) t03 = millis();
                if (intakeDone) {
                    t0 = millis();
                    is = IntakeState::FRONT;
                }
            }
            if (millis() - t0 > 2000) { i++; }
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

/*
 ##     ## #### ########     ########    ###    ########
 ###   ###  ##  ##     ##    ##         ## ##   ##     ##
 #### ####  ##  ##     ##    ##        ##   ##  ##     ##
 ## ### ##  ##  ##     ##    ######   ##     ## ########
 ##     ##  ##  ##     ##    ##       ######### ##   ##
 ##     ##  ##  ##     ##    ##       ##     ## ##    ##
 ##     ## #### ########     ##       ##     ## ##     ##
*/

void autonMidFar(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 10;
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
    Point ptBeforeC1, ptC1, ptBeforeShoot, ptShoot, pivotBeforeC2, ptC2, ptAfterC2, ptAfterC22, ptBalls, ptShoot2, ptPost2;
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
        ptBeforeC1 = Point(0, 30);
        ptC1 = Point(0, 46);
        ptBeforeShoot = Point(-2, 35);
        ptShoot = ptBeforeShoot + polarToRect(6, PI / 4);  // far post
        pivotBeforeC2 = Point(-18, 25);
        ptC2 = Point(-19, 41);
        ptAfterC2 = Point(-19, 35);
        ptAfterC22 = Point(-19, 12);
        turnAfterC2 = -PI;
        ptBalls = Point(-19, 39);
        ptShoot2 = Point(20, 0);  // middle post
        ptPost2 = Point(100, 48);
    }
    Point pt0(0, 0);
    odometry.reset();
    odometry.setA(targetAngle);
    odometry.setX(pt0.x);
    odometry.setY(pt0.y);

    // initialize
    t0 = millis();
    pidFlywheelInit(3.0, 0.1, 500);
    pidDriveLineInit(pt0, ptBeforeC1, true, 0.1, 0);
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
            printf("drv to cap 1 ");
            printDrivePidValues();
            clawPidRunning = true;
            clawPid.target = 0;
            drfbPidRunning = true;
            drfbPid.target = 300;
            if (getDrfb() > drfbPosCloseIntake - 70) is = IntakeState::FRONT;
            bool driveDone = false;
            if (k == 0) {
                if (pidDriveLine()) {
                    pidDriveLineInit(ptBeforeC1, ptC1, true, 0.1, 0);
                    k++;
                }
            } else if (k == 1) {
                if (pidDriveLine()) { driveDone = true; }
            }
            if (driveDone) {
                pidDriveLineInit(ptC1, ptBeforeShoot, false, 0.1, driveT);
                t0 = BIL;
                i++;
            }
        } else if (i == j++) {  // drive to ptBeforeShoot
            printf("drive to ptBeforeShoot ");
            printDrivePidValues();
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 300) is = IntakeState::FRONT_HOLD;
            if (pidDriveLine()) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, true, 0.08, driveT);
                k = 0;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {  // drive to ptShoot, shoot side post flags
            printf("drive to ptShoot ");
            printDrivePidValues();
            bool driveDone = pidDriveLine();
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0 + 50) t02 = millis();
            setDrfb(millis() - t02 < 500 ? -12000 : -1500);
            if (driveDone) {
                if (k == 0) {
                    is = IntakeState::FRONT;
                    t03 = millis();
                    k++;
                } else if (k == 1) {
                    if (millis() - t03 > 2000) {
                        pidDriveLineInit(ptShoot, pivotBeforeC2, false, 0.1, 1);
                        k = 0;
                        i++;
                    }
                }
            } else {
                if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
                if (millis() - t0 > 300) { is = IntakeState::FRONT_HOLD; }
            }
        } else if (i == j++) {  // drive to C2
            printf("drive to C2 ");
            printDrivePidValues();
            is = IntakeState::NONE;
            drfbPidRunning = true;
            if (k == 0) {
                drfbPid.target = drfb18Max - 30;
                if (pidDriveLine()) {
                    pidDriveLineInit(pivotBeforeC2, ptC2, false, 0.1, driveT);
                    k++;
                }
            } else if (k == 1) {
                drfbPid.target = (odometry.getY() > ptC2.y - 1.2) ? drfbPosScrape : 500;
                if (pidDriveLine()) {
                    pidDriveLineInit(ptC2, ptAfterC2, true, 0.1, driveT);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("scrape balls off C2 ");
            printPidValues();
            if (k == 0) {
                drfbPidRunning = false;
                if (getDrfb() < drfbPosScrape) {
                    if (pidDriveLine()) {
                        setDrfb(-1500);
                        pidDriveLineInit(ptAfterC2, ptAfterC22, true, 0.1, 0);
                        k++;
                    }
                } else {
                    setDL(0);
                    setDR(0);
                    setDrfb(-12000);
                }
            } else if (k == 1) {
                drfbPidRunning = true;
                drfbPid.target = drfbPosCloseIntake;
                if (pidDriveLine()) {
                    pidTurnInit(odometry.getA() + turnAfterC2, 0);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("grab Balls ");
            printDrivePidValues();
            if (k == 0) {  // Turn to face balls
                printf("Turn ");
                if (pidTurn()) {
                    pidDriveLineInit(odometry.getPos(), ptBalls, true, 0.1, 0);
                    k++;
                }
            } else if (k == 1) {  // drive to grab balls
                printf("Drive ");
                is = IntakeState::FRONT;
                if (pidDriveLine()) {
                    pidDriveLineInit(ptBalls, ptShoot2, true, 0.1, driveT);
                    t0 = BIL;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive twd ptShoot2 ");
            printDrivePidValues();
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 300) { is = IntakeState::FRONT_HOLD; }
            if (pidDriveLine()) {
                pidFaceInit(ptPost2, true, 300);
                t0 = BIL;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {
            printf("face post 2 ");
            printDrivePidValues();
            drfbPidRunning = false;
            if (getDrfb() > drfbPos0 + 50) t02 = millis();
            setDrfb(millis() - t02 > 500 ? -1500 : -12000);
            if (pidFace()) {
                t0 = millis();
                is = IntakeState::FRONT;
            }
            if (millis() - t0 > 2000) { i++; }
        } else if (i == j++) {
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            stopMotors();
            break;
        }
        clawPid.sensVal = getClaw();
        setClaw(clawPid.update());
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}

void autonomous() {
    odometry.reset();
    setDriveSlew(true);
    if (autoSel_nAuton == 0) {
        stopMotorsBlock();
    } else if (autoSel_nAuton == 1) {
        autonNearMid(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        autonMidFar(autoSel_leftSide);
    }
    stopMotorsBlock();
}
