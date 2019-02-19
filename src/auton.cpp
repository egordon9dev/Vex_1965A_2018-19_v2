#include "Point.hpp"
#include "main.h"
#include "setup.hpp"
/*
----------------------
----    to do     ----
----------------------

- vision

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
using pros::delay;
using std::cout;
using std::endl;

/*
   ###    ##     ## ########  #######  ##    ##     #######
  ## ##   ##     ##    ##    ##     ## ###   ##    ##     ##
 ##   ##  ##     ##    ##    ##     ## ####  ##           ##
##     ## ##     ##    ##    ##     ## ## ## ##     #######
######### ##     ##    ##    ##     ## ##  ####           ##
##     ## ##     ##    ##    ##     ## ##   ###    ##     ##
##     ##  #######     ##     #######  ##    ##     #######
3 flag auton

align fwd/bwd: to tile edge (ignore tabs)
align left/right: 1 finger(3 segments tip to 3rd joint/wrinkle) far from the platform
*/
void auton3(bool leftSide) {
    printf("\n\n\n--------------------   Auton 3 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 150, turnT = 200, driveTBeforeShoot = 500;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL, t02 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false, intakeRunning = true;
    int prevITime = millis();
    int timeBetweenI = 4500;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;

    // tuning setpoints
    Point ptBeforeCap1;
    Point ptAfterCap1;
    Point ptPivot1;
    Point ptShoot1;
    Point ptShoot2;
    Point ptAfterCap2;
    Point ptPivotBeforeBtmFlag;
    Point ptAfterBtmFlag;

    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    if (leftSide) {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptPivot1 = Point(0, -0.5);
        ptShoot1 = Point(-4.5 * sideSign, -0.5);
        ptShoot2 = Point(-25.5 * sideSign, -0.5);
        ptAfterCap2 = Point(-23.5 * sideSign, 20);
        ptPivotBeforeBtmFlag = Point(-23.5 * sideSign, -5.5);
        ptAfterBtmFlag = Point(-47.5 * sideSign, -10);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptPivot1 = Point(0, -4);
        ptShoot1 = Point(-1.5 * sideSign, -4);
        ptShoot2 = Point(-23.75 * sideSign, -4);
        ptAfterCap2 = Point(-23 * sideSign, 18);
        ptPivotBeforeBtmFlag = Point(-24 * sideSign, -8);
        ptAfterBtmFlag = Point(-47.5 * sideSign, -12);
    }
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(-4);

    // initialize
    t0 = millis();
    pidFlywheelInit(2.9, 500);
    odometry.reset();
    pidDriveInit(ptBeforeCap1, 0);
    setDriveSlew(true);
    while (!ctlr.get_digital(DIGITAL_B)) {
        printf("%.2f ", (millis() - autonT0) / 1000.0);
        pros::lcd::print(8, "Time: %d ms", millis() - autonT0);
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 15000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) {
            printf("\ntimeBetweenI exceeded breaking out of auton3....\n");
            break;
        }
        int j = 0;
        odometry.update();
        if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1 ");
            printDrivePidValues();
            drfbPidRunning = true;
            drfbPid.target = drfbPos0;
            clawPidRunning = true;
            clawPid.target = 0;
            is = IntakeState::FRONT;
            if (k == 0) {
                pidDrive();
                if (odometry.getY() > ptBeforeCap1.y - 1 && fabs(getDriveVel()) < 20) k++;
            } else if (k == 1) {
                setDL(-5000);
                setDR(-5000);
                if (odometry.getY() > ptAfterCap1.y) {
                    pidDriveLineInit(ptAfterCap1, ptPivot1, false, 0.1, driveT);
                    timeBetweenI = 4500;
                    t0 - millis();
                    i++;
                }
            }

        } else if (i == j++) {  // drive back
            printf("drive back ");
            printDrivePidValues();
            is = IntakeState::ALTERNATE;
            if (pidDriveLine()) {
                pidDriveLineInit(ptPivot1, ptShoot1, true, 0.03, driveTBeforeShoot);
                timeBetweenI = 3500;
                i++;
            }
        } else if (i == j++) {  // go to pos 1
            printf("go to pos 1 ");
            printDrivePidValues();
            if (pidDrive()) {
                timeBetweenI = 4500;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // load 1
            printf("load 1 ");
            printPidValues();
            pidDrive();
            if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 1000) {  // typical: 10ms
                intakeRunning = false;
                pidIntakeInit(intakeShootTicks, 80);
                i++;
            }
        } else if (i == j++) {  // shoot 1
            printf("shoot 1 ");
            printPidValues();
            pidDrive();
            if (pidIntake()) {
                intakeRunning = true;
                is = IntakeState::ALTERNATE;
                pidFlywheelInit(2.9, 500);
                pidDriveLineInit(ptShoot1, ptShoot2, true, 0.05, driveTBeforeShoot);
                timeBetweenI = 4000;
                i++;
            }
        } else if (i == j++) {  // drive to pos 2
            printf("drive to shoot pos 2 ");
            printDrivePidValues();
            if (pidDrive()) {
                t0 = millis();
                timeBetweenI = 4500;
                i++;
            }
        } else if (i == j++) {  // load 2
            printf("load 2 ");
            printPidValues();
            pidDrive();
            if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 3000) {  // typical: 1400ms
                intakeRunning = false;
                pidIntakeInit(intakeShootTicks, 80);
                i++;
            }
        } else if (i == j++) {  // shoot 2
            printf("shoot 2 ");
            printPidValues();
            pidDrive();
            if (pidIntake()) {
                intakeRunning = true;
                pidFlywheelInit(1.0, 9999);
                is = IntakeState::FRONT;
                timeBetweenI = 4500;
                pidDriveLineInit(ptShoot2, ptAfterCap2, false, 0.07, driveT);
                i++;
            }
        } else if (i == j++) {  // drive twd cap
            printf("drive twd cap 2 ");
            printDrivePidValues();
            drfbPidRunning = false;
            setDrfb(-12000);
            if (pidDriveLine()) {
                k = 0;
                i++;
            }
        } else if (i == j++) {  // flip cap
            int o = 0;
            if (k == o++) {
                printf("flip cap ");
                printPidValues();
                drfbPidRunning = true;
                drfbPid.target = drfb18Max;
                if (getDrfb() > drfbMinClaw0 - 100) k++;
            } else if (k == o++) {
                clawPid.target = claw180;
                printPidValues();
                if (getClaw() > claw180 * 0.55) {
                    pidDriveLineInit(ptAfterCap2, ptPivotBeforeBtmFlag, true, 0.1, driveT);
                    k++;
                }
            } else if (k == o++) {
                setDrfb(-12000);
                printf("drive twd ptPivotBeforeBtmFlag ");
                printDrivePidValues();
                if (pidDriveLine()) {
                    pidDriveLineInit(ptPivotBeforeBtmFlag, ptAfterBtmFlag, true, 0.05, driveT);
                    timeBetweenI = 3000;
                    t0 = millis();
                    i++;
                }
            }
        } else if (i == j++) {  // knock btm flag
            if (millis() - t0 < 1200) {
                drfbPidRunning = false;
                setDrfb(-12000);
            } else {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            printf("pivot + knock bottom flag");
            printDrivePidValues();
            if (pidDrive()) { i++; }
        } else {
            printing = false;
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            i = 99999;
            stopMotors();
        }
        if (clawPidRunning) pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        if (intakeRunning) setIntake(is);
        delay(10);
    }
    stopMotorsBlock();
}
/*
    ###    ##     ## ########  #######  ##    ##    ##
   ## ##   ##     ##    ##    ##     ## ###   ##    ##    ##
  ##   ##  ##     ##    ##    ##     ## ####  ##    ##    ##
 ##     ## ##     ##    ##    ##     ## ## ## ##    ##    ##
 ######### ##     ##    ##    ##     ## ##  ####    #########
 ##     ## ##     ##    ##    ##     ## ##   ###          ##
 ##     ##  #######     ##     #######  ##    ##          ##
align fwd/bwd: to tile edge (ignore tabs)
align left/right: 1 finger(3 segments tip to 3rd joint/wrinkle) far from the platform
*/
void auton4(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 200, turnT = 500;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false;
    clawPid.target = 0;
    int prevITime = millis();
    int timeBetweenI = 4500;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;
    Point ptA, ptB;
    t0 = millis();
    odometry.reset();
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(0);
    ptB = Point(0, 45);
    setDriveSlew(true);
    pidDriveInit(ptB, driveT);
    flywheelPid.target = 2.9;
    drfbPid.target = drfbPos0;
    while (!ctlr.get_digital(DIGITAL_B)) {
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 1500000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) {
            printf("\ntimeBetweenI exceeded breaking out of auton4....\n");
            break;
        }
        int j = 0;
        odometry.update();
        if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1");
            printDrivePidValues();
            drfbPidRunning = true;
            drfbPid.target = drfbPos0;
            clawPidRunning = true;
            clawPid.target = 0;
            is = IntakeState::FRONT;
            if (odometry.getY() > 30) driveLim = 4000;
            if (pidDrive()) {
                driveLim = 12000;
                ptB = Point(0, 32);
                pidDriveInit(ptB, driveT);
                timeBetweenI = 4500;
                i++;
            }
        } else if (i == j++) {  // drive back
            printf("drive back from cap ");
            printDrivePidValues();
            if (pidDrive()) {
                is = IntakeState::NONE;
                targetAngle += sideSign * (PI / 2);
                pidTurnInit(targetAngle, 200);
                timeBetweenI = 4500;
                i++;
            }
        } else if (i == j++) {  // turn
            is = getISLoad();
            printf("turn 90 deg ");
            printDrivePidValues();
            if (pidTurn()) {
                ptB.x = sideSign * 20;
                pidDriveInit(ptB, driveT);
                timeBetweenI = 4500;
                i++;
            }
        } else if (i == j++) {  // drive twd pivot
            is = getISLoad();
            printf("drive to pivot ");
            printDrivePidValues();
            if (pidDrive()) {
                i++;
                targetAngle += sideSign * (PI / 2);
                pidTurnInit(targetAngle, turnT);
            }
        } else if (i == j++) {  // pivot

            is = getISLoad();
            printf("drive to pivot ");
            printDrivePidValues();
            if (pidTurn()) {
                i++;
                ptB.y = 47;
                pidDriveInit(ptB, driveT);
            }
        } else if (i == j++) {  // drive twd cap
            is = getISLoad();
            printf("drive twd cap ");
            printDrivePidValues();
            drfbPidRunning = false;
            if (getDrfbCurrent() < 1000) {
                setDrfb(-2000);
            } else {
                setDrfb(0);
            }
            if (pidDrive()) {
                drfbPidRunning = true;
                drfbPid.target = drfb18Max - 50;
                clawPid.target = claw180;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            printf("pick up cap ");
            printPidValues();
            is = getISLoad();
            setDL(0);
            setDR(0);
            if (millis() - t0 > 300) {
                i++;
                timeBetweenI = 4500;
                ptB.y = 25;
                ptB.x = sideSign * 18;
                driveLim = 6000;
                drfbPid.target = drfbPos1 + 400;
                pidDriveInit(ptB, driveT);
            }
        } else if (i == j++) {  // drive back
            is = getISLoad();
            printf("drive back ");
            printDrivePidValues();
            is = getISLoad();
            if (pidDrive()) {
                targetAngle += sideSign * (-PI / 2);
                pidTurnInit(targetAngle, turnT);
                timeBetweenI = 4500;
                driveLim = 8000;
                i++;
            }
        } else if (i == j++) {  // turn
            printf("turn twd pipe ");
            printDrivePidValues();
            is = getISLoad();
            if (pidTurn()) {
                ptB.x = sideSign * 26;
                pidDriveInit(ptB, driveT);
                timeBetweenI = 3000;
                driveLim = 6000;
                i++;
            }
        } else if (i == j++) {  // funnel against pipe
            printf("funnel against pipe ");
            printDrivePidValues();
            is = getISLoad();
            if (pidDrive() || millis() - t0 > 1000) {
                ptB.x = sideSign * 25;
                pidDriveInit(ptB, driveT);
                i++;
            }
        } else if (i == j++) {
            printf("align cap with pipe ");
            printDrivePidValues();
            is = getISLoad();
            if (pidDrive()) {
                drfbPid.target = drfbPos1;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // set down cap
            printf("set down cap ");
            printDrivePidValues();
            is = getISLoad();
            setDL(-1000);
            setDR(-1000);
            if (millis() - t0 > 800) {
                i++;
                t0 = millis();
                ptB.x = sideSign * 12;
                pidDriveInit(ptB, driveT);
            }
        } else if (i == j++) {  // drive back
            printf("drive back from pipe ");
            printDrivePidValues();
            is = getISLoad();
            if (pidDrive()) {
                driveLim = 12000;
                timeBetweenI = 4500;
                drfbPidRunning = false;
                targetAngle += sideSign * 0.07;
                pidTurnInit(targetAngle, turnT);
                i++;
            }
        } else if (i == j++) {  // turn to face flag
            printf("turn to face flag pos 1 ");
            printDrivePidValues();
            is = getISLoad();
            if (getDrfb() < drfbPos0 + 200) {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            } else {
                setDrfb(-12000);
            }
            if (pidTurn()) {
                i++;
                t0 = millis();
            }
        } else if (i == j++) {  // shoot ball 1
            printf("shoot ball 1 ");
            setDL(0);
            setDR(0);
            is = IntakeState::BACK;
            if (millis() - t0 > 700) {
                is = IntakeState::NONE;
                ptB = ptB - polarToRect(10, targetAngle);
                pidDriveInit(ptB, driveT);
                i++;
            }
        } else if (i == j++) {
            printf("drive to pos 2 ");
            if (pidDrive()) {
                i++;
                k = 0;
            }
        } else if (i == j++) {  // shoot ball 2
            printf("shoot ball 2 ");
            int o = 0;
            if (k == o++) {
                is = getISLoad();
                t0 = millis();
                if (is == IntakeState::NONE) k++;
            } else if (k == o++) {
                is = IntakeState::BACK;
            }
            setDL(0);
            setDR(0);
            if (millis() - t0 > 1000) {
                is = IntakeState::NONE;
                i++;
            }
        } else if (i == j++) {
            printing = false;
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            i = 99999;
            stopMotors();
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

void testAuton() {
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(0);
    int t0 = millis();
    pidDriveInit(Point(0, -20), 99);
    while (!ctlr.get_digital(DIGITAL_B)) {
        odometry.update();
        pidDrive();
        printDrivePidValues();
        delay(10);
    }
}
void autonomous() {
    odometry.reset();
    setDriveSlew(true);
    auton3(true);
    stopMotorsBlock();
}
