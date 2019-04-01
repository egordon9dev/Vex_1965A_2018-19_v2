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
    const int driveT = 150, driveTBeforeShoot = 500;
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
        ptBeforeCap1 = Point(0, 26);
        ptAfterCap1 = Point(0, 34.5);
        ptPivot1 = Point(0, 0.25);
        ptShoot1 = Point(-5.25 * sideSign, 0.5);
        ptShoot2 = Point(-26.6 * sideSign, 1);
        ptAfterCap2 = Point(-24.5 * sideSign, 10);
        ptPivotBeforeBtmFlag = Point(-23.5 * sideSign, -4);
        ptAfterBtmFlag = Point(-52 * sideSign, -5);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeCap1 = Point(0, 34.5);
        ptAfterCap1 = Point(0, 37.5);
        ptPivot1 = Point(0, -0.1);
        ptShoot1 = Point(-0.5 * sideSign, -0.1);
        ptShoot2 = Point(-24.3 * sideSign, -0.1);
        ptAfterCap2 = Point(-25 * sideSign, 11);
        ptPivotBeforeBtmFlag = Point(-24 * sideSign, -4);
        ptAfterBtmFlag = Point(-50 * sideSign, -9.5);
    }
    Point origin = Point(0, -4);
    odometry.setA(-PI / 2);
    odometry.setX(origin.x);
    odometry.setY(origin.y);
    odometry.reset();

    // initialize
    t0 = millis();
    pidFlywheelInit(sShotSpeed, 0.1, 500);
    pidDriveLineInit(origin, ptBeforeCap1, true, 0.1, 0);  //<--- keep this 0 wait!!
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
        if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1 ");
            printDrivePidValues();
            drfbPidRunning = true;
            drfbPid.target = drfbPos0;
            clawPidRunning = true;
            clawPid.target = 0;
            is = IntakeState::FRONT;
            if (odometry.getY() > ptBeforeCap1.y - 8) {
                DLSlew.slewRate = DRSlew.slewRate = 40;
                setMaxAErr(9999);
            }
            if (k == 0) {
                if (pidDriveLine()) {
                    t0 = millis();
                    k++;
                }
            } else if (k == 1) {
                setDL(-12000);
                setDR(-12000);
            }
            if (odometry.getY() > ptAfterCap1.y) {
                pidDriveLineInit(ptAfterCap1, ptPivot1, false, 9999, driveT);
                timeBetweenI = 4500;
                driveLim = 4000;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // drive back
            printf("drive back ");
            printDrivePidValues();
            if (millis() - t0 > 1800) is = IntakeState::ALTERNATE;
            if (odometry.getY() < ptAfterCap1.y - 6) {
                setMaxAErr(0.1);
                driveLim = 12000;
                setDriveSlew(true);
            }
            if (pidDriveLine()) {
                is = IntakeState::ALTERNATE;
                pidDriveLineInit(ptPivot1, ptShoot1, true, 0.08, driveTBeforeShoot);
                timeBetweenI = 3500;
                i++;
            }
        } else if (i == j++) {  // go to pos 1
            printf("go to pos 1 ");
            printDrivePidValues();
            if (pidDriveLine()) {
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
                pidFlywheelInit(sShotSpeed, 0.1, 500);
                pidDriveLineInit(ptShoot1, ptShoot2, true, 0.08, driveTBeforeShoot);
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
            if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 1900) {  // typical: 1400ms
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
                pidFlywheelInit(1.0, 0.1, 9999);
                is = IntakeState::FRONT;
                timeBetweenI = 4500;
                pidDriveLineInit(ptShoot2, ptAfterCap2, false, 0.08, driveT);
                i++;
            }
        } else if (i == j++) {  // drive twd cap
            printf("drive twd cap 2 ");
            printDrivePidValues();
            drfbPidRunning = false;
            setDrfb(-12000);
            printf("<%.2f / %.2f>", getDrfb(), drfbPid.target);
            if (pidDriveLine()) {
                k = 0;
                i++;
            }
        } else if (i == j++) {  // flip cap
            int o = 0;
            if (k == o++) {
                printf("flip cap ");
                printPidValues();
                drfbPidRunning = false;
                // drfbPid.target = drfb18Max;
                setDrfb(12000);
                if (getDrfb() > drfbMinClaw0 - 100) {
                    drfbPid.target = drfb18Max;
                    drfbPidRunning = true;
                    k++;
                }
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
                    pidDriveLineInit(ptPivotBeforeBtmFlag, ptAfterBtmFlag, true, 0.08, driveT);
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
                }lksand
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

2 balls middle post, 1 cap high post, 1 low scored cap

Alignment tool: 3 3/8 + 1/32
*/

void auton5(bool leftSide) {
    printf("\n\n\n--------------------   Auton 5 ---------------------\n\n\n");
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    double targetAngle = -PI / 2;
    const int driveT = 85, driveTBeforeShoot = 300;
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
    driveLim = 12000;

    // tuning setpoints
    Point ptBeforeCap1;
    Point ptAfterCap1;
    Point ptBeforeShoot;
    Point ptShoot;
    Point ptAfterCap2;
    Point ptBeforePost;
    Point ptBeyondPost;
    double fwSpeed1, fwSpeed2;

    fwSpeed1 = 2.65;
    fwSpeed2 = 2.85;

    /*************************************************
    ***********     Left (Red) Side     **************
    **************************************************/
    if (leftSide) {
        // ptAfterCap1 = Point(0, 41.5);
        // ptBeforeShoot = Point(0, 6.67);
        // ptShoot = Point(17 * sideSign, 0);
        // ptAfterCap2 = Point(21 * sideSign, 46);
        // ptBeforePost = Point(11.5 * sideSign, 20.5);
        // ptBeyondPost = Point(34 * sideSign, 20.5);

        ptBeforeCap1 = Point(0, 38.5);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(0, 6.965);
        ptShoot = Point(17 * sideSign, 0);
        ptAfterCap2 = Point(22.9 * sideSign, 43.3);
        ptBeforePost = Point(11.5 * sideSign, 19.5);
        ptBeyondPost = Point(34 * sideSign, 19.5);
    }

    /*************************************************
    ***********     Right (Blue) Side     ************
    **************************************************/
    else {
        ptBeforeCap1 = Point(0, 38.5);
        ptAfterCap1 = Point(0, 41.5);
        ptBeforeShoot = Point(0, 6.7);
        ptShoot = Point(17 * sideSign, 0);
        ptAfterCap2 = Point(22.80 * sideSign, 45.01);
        ptBeforePost = Point(11.5 * sideSign, 20.5);
        ptBeyondPost = Point(34 * sideSign, 20.5);
    }
    Point origin(0, 0);
    odometry.reset();
    odometry.setA(-PI / 2);
    odometry.setX(origin.x);
    odometry.setY(origin.y);

    Point p1;
    // initialize
    t0 = BIL;
    pidFlywheelInit(fwSpeed2, 0.1, 700);
    pidDriveLineInit(origin, ptBeforeCap1, true, 0.1, 0);  //<--- keep this 0 wait!!
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
        if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1 %d ", driveLim);
            printDrivePidValues();
            drfbPidRunning = true;
            drfbPid.target = drfbPos0;
            clawPidRunning = true;
            clawPid.target = 0;
            is = IntakeState::FRONT;
            if (odometry.getY() > ptBeforeCap1.y - 14) {
                DLSlew.slewRate = DRSlew.slewRate = 40;
                setMaxAErr(9999);
            }
            if (k == 0) {
                if (pidDriveLine()) k++;
            } else if (k == 1) {
                setDL(-12000);
                setDR(-12000);
            }
            if (odometry.getY() > ptAfterCap1.y) {
                timeBetweenI = 4500;
                driveLim = 4000;
                pidDriveLineInit(ptAfterCap1, ptBeforeShoot, false, 9999, driveT);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // drive back
            printf("drive to ptBeforShoot ");
            printDrivePidValues();
            if (millis() - t0 > 1800) is = IntakeState::ALTERNATE;
            if (odometry.getY() < ptAfterCap1.y - 6) {
                setMaxAErr(0.1);
                driveLim = 12000;
                setDriveSlew(true);
            }
            if (pidDriveLine()) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, false, 0.08, driveTBeforeShoot);
                timeBetweenI = 4500;
                is = IntakeState::ALTERNATE;
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
                    pidFlywheelInit(fwSpeed1, 0.1, 800);
                    t0 = millis();
                    k++;
                }
            } else if (k == 2) {
                printf("load 2 ");
                printPidValues();
                if ((isBallIn() && isPidFlywheelDone()) || millis() - t0 > 2500) {
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
                    pidFlywheelInit(1.0, 0.1, 9999);
                    pidDriveLineInit(ptShoot, ptAfterCap2, false, 0.1, 0);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            if ((odometry.getPos() - ptAfterCap2).mag() < 25) drfbPidRunning = false;
            if (!drfbPidRunning) setDrfb(-12000);
            printf("drive to cap 2 ");
            if (k == 0) {
                if (pidDriveLine()) {
                    k = 0;
                    i++;
                }
            }
            printDrivePidValues();
        } else if (i == j++) {
            printf("drive to ptBeforePost ");
            printDrivePidValues();
            if (k == 0) {
                pidDriveLine();
                drfbPidRunning = false;
                setDrfb(12000);
                if (getDrfb() > drfbPos0 + 100) {
                    drfbPidRunning = true;
                    drfbPid.target = drfbPos0 + 170;
                    pidDriveLineInit(ptAfterCap2, ptBeforePost, true, 0.15, driveT);
                    driveLim = 8500;
                    k++;
                }
            } else if (k == 1) {
                if (pidDriveLine()) {
                    pidDriveLineInit(ptBeforePost, ptBeyondPost, false, 0.15, driveT);
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive to post ");
            printDrivePidValues();
            if (k == 0) {
                printf(" turn ");
                pidDriveLine();
                if ((ptBeyondPost - ptBeforePost).angleBetween(polarToRect(1, odometry.getA())) < 0.15) {
                    k++;
                    driveLim = 5000;
                    t0 = millis();
                }
            } else if (k == 1) {
                drfbPid.target = drfbPos0;
                printf(" bump ");
                pidDriveLine();
                if (millis() - t0 > 400 && getDriveVel() < 10) { k++; }
            } else if (k == 2) {
                printf(" lift ");
                setDL(-1000);
                setDR(-1000);
                if (getDrfb() > drfbMinClaw1 - 50) clawPid.target = claw180;
                drfbPid.target = drfbPos1Plus;
                if (getDrfb() > drfbPos1 - 100) {
                    driveLim = 5000;
                    t0 = millis();
                    p1 = odometry.getPos();
                    k++;
                }
            } else if (k == 3) {
                printf(" funnel ");
                if (fabs(odometry.getX()) > fabs(p1.x) + 3.5) drfbPid.target = drfbPos1;
                if (millis() - t0 > 520 && getDriveVel() < 15) {
                    p1 = odometry.getPos();
                    k++;
                } else {
                    pidDriveLine();
                }
            } else if (k == 4) {
                printf("set down cap");
                setDL(0);
                setDR(0);
                drfbPidBias = -3000;
                drfbPid.target = drfbPos1;
                if (getDrfb() < drfbPos1 + 30) {
                    drfbPidBias = 0;
                    driveLim = 8500;
                    k = 0;
                    i++;
                }
            }
        } else if (i == j++) {
            printf("drive back ");
            printDrivePidValues();
            if (k == 0) {
                if (fabs(odometry.getX()) > fabs(p1.x) - 11) {
                    setDL(-8500);
                    setDR(-8500);
                } else {
                    pidDriveLineInit(odometry.getPos(), odometry.getPos() + Point(-0.1 * sideSign, 0), true, 0.3, 9999);
                    k++;
                }
            } else if (k == 1) {
                drfbPid.target = drfbPos0;
                pidDriveLine();
            }
            if (getDrfb() < drfbPos0 + 80) i++;
        } else {
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
        pidDrive();
        printDrivePidValues();
        delay(10);
    }
}

/*
  ######  ##    ## #### ##       ##        ######
 ##    ## ##   ##   ##  ##       ##       ##    ##
 ##       ##  ##    ##  ##       ##       ##
  ######  #####     ##  ##       ##        ######
       ## ##  ##    ##  ##       ##             ##
 ##    ## ##   ##   ##  ##       ##       ##    ##
  ######  ##    ## #### ######## ########  ######
*/

void autonSkills() {}

void autonomous() {
    odometry.reset();
    setDriveSlew(true);
    if (autoSel_nAuton == 0) {
        while (1) { delay(10); }
    } else if (autoSel_nAuton == 1) {
        auton3(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        auton5(autoSel_leftSide);
    }
    stopMotorsBlock();
}
