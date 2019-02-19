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
align fwd/bwd: to tile edge (ignore tabs)
align left/right: 1 finger(3 segments tip to 3rd joint/wrinkle) far from the platform

13 pts
Flag Side: 3 flag + 1 cap auton
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

12 pts
Cap Side: 1 low cap, 1 high cap, 2 left side high flags
*/
/*void auton4(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 150, turnT = 200;
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
    Point ptBeforeShoot;
    Point ptAfterShoot;
    Point ptAfterCap2;
    Point ptBeforePost;
    Point ptPost;
    Point ptAfterPost;
    int fwSpeed1, fwSpeed2;

    fwSpeed1 = dShotSpeed1;
    fwSpeed2 = dShotSpeed2;
    /*************************************************
    ***********     Left (Red) Side     ************
    ************************************************** /
    if (leftSide) {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(16 * sideSign, -1);
        ptShoot = Point(27 * sideSign, -1);
        ptAfterShoot = Point(24 * sideSign, -1);
        ptAfterCap2 = Point(24 * sideSign, 41);
        ptBeforePost = Point(24 * sideSign, 18);
        ptPost = Point(26 * sideSign, 18);
        ptAfterPost = Point(12 * sideSign, 18);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    ************************************************** /
    else {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(16 * sideSign, -1);
        ptShoot = Point(27 * sideSign, -1);
        ptAfterShoot = Point(24 * sideSign, -1);
        ptAfterCap2 = Point(24 * sideSign, 41);
        ptBeforePost = Point(24 * sideSign, 18);
        ptPost = Point(26 * sideSign, 18);
        ptAfterPost = Point(12 * sideSign, 18);
    }
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(0);

    // initialize
    t0 = millis();
    pidFlywheelInit(fwSpeed2, 500);
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
                    timeBetweenI = 4500;
                    i++;
                }
            }

        } else if (i == j++) {  // turn
            printf("turn");
            if (leftSide) {
                setDL(6000);
                setDR(12000);
            } else {
                setDL(12000);
                setDR(6000);
            }
            is = IntakeState::ALTERNATE;
            if (fabs(polarToRect(1, odometry.getA()).angleBetween(ptBeforeShoot - odometry.getPos())) < 0.1) {
                pidDriveLineInit(odometry.getPos(), ptBeforeShoot, false, 0.1, driveT);
                timeBetweenI = 3500;
                i++;
            }
        } else if (i == j++) {  // drive back
            printf("drive to ptBeforShoot ");
            printDrivePidValues();
            if (pidDriveLine()) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, false, 0.05, 300));
                timeBetweenI = 3000;
                i++;
            }
        } else if (i == j++) {
            printf("drive to ptShoot ");
            printDrivePidValues();
            if (pidDriveLine()) {
                timeBetweenI = 6000;
                i++;
                t0 = millis;
                k = 0;
            }
        } else if (i == j++) {
            if (millis() - t0 < 300) {
                setDL(4000);
                setDR(4000);
            } else {
                setDL(2500);
                setDR(2500);
            }
            if (k == 0) {
                printf("load 1 ");
                printPidValues();
                if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 1000) {
                    intakeRunning = false;
                    pidIntakeInit(intakeShootTicks, 80);
                    k++;
                }
            } else if (k == 1) {
                printf("shoot 1 ");
                printPidValues();
                if (pidIntake()) {
                    intakeRunning = true;
                    is = IntakeState::ALTERNATE;
                    pidFlywheelInit(fwSpeed1, 500);
                    t0 = millis();
                    k++;
                }
            } else if (k == 2) {
                printf("load 2 ");
                printPidValues();
                if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 3000) {
                    intakeRunning = false;
                    pidIntakeInit(intakeShootTicks, 80);
                    k++;
                }
            } else if (k == 3) {
                printf("shoot 2 ");
                printPidValues();
                if (pidIntake()) {
                    intakeRunning = true;
                    is = IntakeState::NONE;
                    pidFlywheelInit(1.0, 9999);
                    pidDriveLineInit(odometry.getPos(), ptAfterShoot, true, 0.1, 0);
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive to ptAfterPost");
            printPidDriveValues();
            if (odometry.getY() > ptAfterCap.y - 18) {
                drfbPidRunning = false;
                setDrfb(-12000);
            } else {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            if (pidDriveLine()) {
                pidDriveLineInit(ptAfterShoot, ptAfterCap2, false, 0.1, driveT);
                i++;
            }
        } else if (i == j++) {
            printf("drive to ptAfterPost");
            printPidDriveValues();
            if (pidDriveLine()) {
                pidDriveLineInit(ptAfterShoot, ptAfterCap2, false, 0.1, driveT);
                i++;
            }
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
}*/
/*
    ###    ##     ## ########  #######  ##    ## ########
   ## ##   ##     ##    ##    ##     ## ###   ## ##
  ##   ##  ##     ##    ##    ##     ## ####  ## ##
 ##     ## ##     ##    ##    ##     ## ## ## ## #######
 ######### ##     ##    ##    ##     ## ##  ####       ##
 ##     ## ##     ##    ##    ##     ## ##   ### ##    ##
 ##     ##  #######     ##     #######  ##    ##  ######

8 pts
*/

void auton5(bool leftSide) {
    printf("\n\n\n--------------------   Auton 4 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 150, turnT = 200, driveTBeforeShoot = 400;
    IntakeState is = IntakeState::NONE;
    int t0 = BIL, t02 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false, intakeRunning = true;
    int prevITime = millis();
    int timeBetweenI = 4500;
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    const int autonT0 = millis();

    // tuning setpoints
    Point ptBeforeCap1;
    Point ptAfterCap1;
    Point ptBeforeShoot;
    Point ptShoot;
    Point ptAfterCap2;
    Point ptBeforePost;
    Point ptPost;
    Point ptBeyondPost;
    Point ptAfterPost;
    double fwSpeed1, fwSpeed2;

    fwSpeed1 = 2.58;
    fwSpeed2 = 2.85;
    /*************************************************
    ***********     Left (Red) Side     ************
    **************************************************/
    if (leftSide) {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(0, 21);
        ptShoot = Point(18 * sideSign, 16.5);
        ptAfterCap2 = Point(23 * sideSign, 47);
        ptBeforePost = Point(16 * sideSign, 20);
        ptPost = Point(24 * sideSign, 20);
        ptBeyondPost = ptPost + Point(6 * sideSign, 0);
        ptAfterPost = Point(16 * sideSign, 20);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeCap1 = Point(0, 27);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(0, 24);
        ptShoot = Point(12 * sideSign, 18);
        ptAfterCap2 = Point(25 * sideSign, 40);
        ptBeforePost = Point(18 * sideSign, 24);
        ptPost = Point(26 * sideSign, 24);
        ptBeyondPost = ptPost + Point(6 * sideSign, 0);
        ptAfterPost = Point(18 * sideSign, 24);
    }
    Point origin(0, 0);
    odometry.setA(-PI / 2);
    odometry.setX(origin.x);
    odometry.setY(origin.y);
    odometry.reset();

    // initialize
    t0 = millis();
    pidFlywheelInit(fwSpeed2, 500);
    pidDriveLineInit(origin, ptBeforeCap1, true, 0.1, 0);
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
                pidDriveLine();
                if (odometry.getY() > ptBeforeCap1.y - 5 && fabs(getDriveVel()) < 40) {
                    pidDriveLineInit(ptBeforeCap1, ptAfterCap1, true, 0.1, 0);
                    driveLim = 3500;
                    k++;
                }
            } else if (k == 1) {
                pidDriveLine();
                if (odometry.getY() > ptAfterCap1.y - 0.5) {
                    driveLim = 12000;
                    timeBetweenI = 3000;
                    pidDriveLineInit(ptAfterCap1, ptBeforeShoot, false, 0.1, driveT);
                    i++;
                }
            }

        } else if (i == j++) {  // drive back
            printf("drive to ptBeforShoot ");
            printDrivePidValues();
            is = IntakeState::ALTERNATE;
            if (pidDriveLine()) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, false, 0.05, driveTBeforeShoot);
                timeBetweenI = 3000;
                i++;
            }
        } else if (i == j++) {
            printf("drive to ptShoot ");
            printDrivePidValues();
            if (pidDriveLine()) {
                timeBetweenI = 6000;
                i++;
                t0 = millis();
                k = 0;
            }
        } else if (i == j++) {
            pidDriveLine();
            if (k == 0) {
                printf("load 1 ");
                printPidValues();
                if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 1500) {
                    intakeRunning = false;
                    pidIntakeInit(intakeShootTicks, 80);
                    k++;
                }
            } else if (k == 1) {
                printf("shoot 1 ");
                printPidValues();
                if (pidIntake()) {
                    intakeRunning = true;
                    is = IntakeState::ALTERNATE;
                    pidFlywheelInit(fwSpeed1, 500);
                    t0 = millis();
                    k++;
                }
            } else if (k == 2) {
                printf("load 2 ");
                printPidValues();
                if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 3000) {
                    intakeRunning = false;
                    pidIntakeInit(intakeShootTicks, 80);
                    k++;
                }
            } else if (k == 3) {
                printf("shoot 2 ");
                printPidValues();
                if (pidIntake()) {
                    intakeRunning = true;
                    is = IntakeState::NONE;
                    pidFlywheelInit(1.0, 9999);
                    pidDriveLineInit(ptShoot, ptAfterCap2, false, 0.1, driveT);
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive to cap 2 ");
            printDrivePidValues();
            if ((odometry.getPos() - ptAfterCap2).mag() < 18) {
                drfbPidRunning = false;
                setDrfb(-12000);
            } else {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            if ((odometry.getPos() - ptAfterCap2).mag() < 16) driveLim = 8000;
            if (pidDriveLine()) {
                drfbPidRunning = true;
                drfbPid.target = drfbPos1Plus;
                driveLim = 7000;
                pidDriveLineInit(ptAfterCap2, ptBeforePost, true, 0.1, driveT);
                i++;
            }
        } else if (i == j++) {
            printf("drive to ptBeforePost ");
            if (getDrfb() > drfbMinClaw1) clawPid.target = claw180;
            printDrivePidValues();
            if (pidDriveLine()) {
                clawPid.target = claw180;
                pidDriveLineInit(ptBeforePost, ptBeyondPost, false, 0.1, driveT);
                driveLim = 7000;
                t0 = millis();
                k = 0;
                i++;
            }
        } else if (i == j++) {
            printf("drive to post ");
            if (odometry.getX() * sideSign > ptPost.x) driveLim = 2500;
            printDrivePidValues();
            pidDriveLine();
            if (k == 0) {
                if (millis() - t0 > 700 && getDriveVel() < 10) { k++; }
            } else if (k == 1) {
                drfbPid.target = drfbPos1;
                if (getDrfb() < drfbPos1 + 70) {
                    pidDriveInit(ptAfterPost, driveT);
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive back ");
            printDrivePidValues();
            if (pidDriveLine()) { i++; }
        } else if (i == j++) {
            printf("lwr drfb");
            printPidValues();
            drfbPid.target = drfbPos0;
            if (getDrfb() < drfbPos0 + 80) i++;
        } else if (i == j++) {
            if (i == 12345) printf("\n\nAUTON TIMEOUT\n");
            stopMotors();
            break;
        }
        if (clawPidRunning) pidClaw();
        pidFlywheel();
        if (drfbPidRunning) pidDrfb();
        if (intakeRunning) setIntake(is);
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