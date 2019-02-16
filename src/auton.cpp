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
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(-4);
    double targetAngle = -PI / 2;
    const int driveT = 300, turnT = 500;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL, t02 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false;
    int prevITime = millis();
    int timeBetweenI = 4000;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;
    Point ptA, ptB, ptC;
    int enc0;
    while (!ctlr.get_digital(DIGITAL_B)) {
        printf("%.2f ", (millis() - autonT0) / 1000.0);
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 1500000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) {
            printf("\ntimeBetweenI exceeded breaking out of auton3....\n");
            break;
        }
        int j = 0;
        odometry.update();
        if (i == j++) {
            printf("initializing ");
            printPidValues();
            t0 = millis();
            ptB = Point(0, 41);
            pidDriveInit(ptB, 400);
            enc0 = getDrfbEncoder();
            flywheelPid.target = 2.9;
            odometry.reset();
            i++;
            timeBetweenI = 4500;
        } else if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1 ");
            printDrivePidValues();
            drfbPidRunning = true;
            drfbPid.target = drfbPos0;
            clawPidRunning = true;
            clawPid.target = 0;
            is = IntakeState::FRONT;
            if (odometry.getY() > 30) driveLim = 4000;
            if (pidDrive()) {
                driveLim = 12000;
                ptA = Point(0, 0);
                ptB = Point(sideSign * -50, 0);
                pidDriveInit(ptA + 7 * (ptB - ptA).unit(), driveT);
                timeBetweenI = 4500;
                t0 - millis();
                i++;
            }
        } else if (i == j++) {  // drive back
            printf("drive back ");
            printDrivePidValues();
            if (millis() - t0 > 600) is = IntakeState::ALTERNATE;
            if (pidDrive()) {
                pidFaceInit(ptB, true, 1000);
                timeBetweenI = 3500;
                i++;
            }
        } else if (i == j++) {  // turn to face pos 1
            printf("turn to face pos 1 ");
            printDrivePidValues();
            is = IntakeState::ALTERNATE;
            if (pidFace()) {
                timeBetweenI = 3500;
                k = 0;
                t02 = t0 = BIL;
                i++;
            }
        } else if (i == j++) {  // shoot 1
            if (k == 0) {
                is = IntakeState::ALTERNATE;
                // time could be saved here by doing this in previous steps
                if (fabs(flywheelPid.sensVal - flywheelPid.target) < 0.05 && t02 > millis()) t02 = millis();
                if (isBallIn() && millis() - t02 > 500) {
                    t0 = millis();
                    is = IntakeState::BACK_SLOW;
                    k++;
                }
                printf("load 1 ");
            } else {
                printf("shoot 1 ");
            }
            printPidValues();
            setDL(0);
            setDR(0);
            if (millis() - t0 > 400) {
                is = IntakeState::FRONT;
                pidDriveInit(ptA + 30 * (ptB - ptA).unit(), driveT);
                timeBetweenI = 3500;
                i++;
            }
        } else if (i == j++) {  // drive to pos 2
            printf("drive to shoot pos 2 ");
            printDrivePidValues();
            if (pidDrive()) {
                i++;
                k = 0;
                t0 = BIL;
            }
        } else if (i == j++) {  // shoot 2
            if (k == 0) {
                is = IntakeState::ALTERNATE;
                if (fabs(flywheelPid.sensVal - flywheelPid.target) < 0.05 && t02 > millis()) t02 = millis();
                if (isBallIn() && millis() - t02 > 500) {
                    t0 = millis();
                    is = IntakeState::BACK_SLOW;
                    k++;
                }
                printf("load 2 ");
            } else {
                printf("shoot 2 ");
            }
            printPidValues();
            setDL(0);
            setDR(0);
            if (millis() - t0 > 400) {
                timeBetweenI = 4500;
                // pidDriveInit(ptB, 0);
                ptA = Point(-25 * sideSign, 18);
                ptB = Point(-50 * sideSign, -6.5);
                pidDriveLineInit(ptA, false, 0.05, driveT);
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
            printf("flip cap ");
            printPidValues();
            if (k == o++) {
                drfbPidRunning = true;
                drfbPid.target = drfb18Max;
                if (getDrfb() > drfbMinClaw0 - 100) k++;
            } else if (k == o++) {
                clawPid.target = claw180;
                if (getClaw() > claw180 * 0.55) { drfbPidRunning = false; }
                if (!drfbPidRunning) setDrfb(-12000);
                if (getDrfb() < drfbPos0 + 10) {
                    int targetDL, targetDR;
                    if (leftSide) {
                        targetDL = -16;
                        targetDR = -35;
                    } else {
                        targetDL = -35;
                        targetDR = -16;
                    }
                    pidSweepInit(targetDL, targetDR, 0);
                    k++;
                }
            } else if (k == o++) {
                setDrfb(-12000);
                if (pidSweep()) {
                    setDL(0);
                    setDR(0);
                    i++;
                }
            }
        } else {
            printing = false;
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            i = 99999;
            stopMotors();
        }
        if (clawPidRunning) pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        setIntake(is);
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
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(0);
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
    int timeBetweenI = 4000;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();
    bool printing = true;

    Point ptA, ptB;
    int enc0;
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
        printf(" fly %f/%f ", flywheelPid.sensVal, flywheelPid.target);
        if (i == j++) {
            printf("initializing ");
            printPidValues();
            t0 = millis();
            ptB = Point(0, 47);
            pidDriveInit(ptB, 400);
            enc0 = getDrfbEncoder();
            flywheelPid.target = 2.9;
            drfbPid.target = drfbPos0;
            i++;
            timeBetweenI = 4500;
        } else if (i == j++) {  // drive fwd, pick up ball under cap
            printf("drive fwd to cap ");
            printDrivePidValues();
            is = IntakeState::FRONT;
            if (pidDrive()) {
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
    setDriveSlew(true);
    auton3(true);
    stopMotorsBlock();
}
