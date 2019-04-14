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
void auton4(bool leftSide) {
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
        ptC1 = Point(0, 45);
        ptBeforeShoot = Point(-4, 0);
        ptShoot = Point(6, 0.5);
        pivotBeforeC2 = Point(-18, 12);
        ptC2 = Point(-19, 41);
        ptAfterC2 = Point(-19, 35);
        ptAfterC22 = Point(-19, 12);
        turnAfterC2 = -PI;
        ptBalls = Point(-19, 39);
        ptShoot2 = Point(20, 0);
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
            printf("\ntimeBetweenI exceeded breaking out of auton3....\n");
            break;
        }
        int j = 0;
        if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1 ");
            printDrivePidValues();
            clawPidRunning = true;
            clawPid.target = 0;
            drfbPidRunning = true;
            drfbPid.target = drfbPosCloseIntake + 100;
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
        } else if (i == j++) {
            printf("drive twd ptBeforeShoot ");
            printDrivePidValues();
            if (isTopBallIn() && isBtmBallIn() && t0 > millis()) t0 = millis();
            if (millis() - t0 > 300) is = IntakeState::FRONT_HOLD;
            if (pidDriveLine()) {
                pidDriveLineInit(ptBeforeShoot, ptShoot, true, 0.08, driveT);
                k = 0;
                t02 = millis();
                i++;
            }
        } else if (i == j++) {
            printf("drive twd ptShoot ");
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
        } else if (i == j++) {
            printf("drive twd C2 ");
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
                drfbPid.target = 500;
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
            if (k == 0) {
                printf("Turn ");
                if (pidTurn()) {
                    pidDriveLineInit(odometry.getPos(), ptBalls, true, 0.1, 0);
                    k++;
                }
            } else if (k == 1) {
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
                // DLSlew.slewRate = DRSlew.slewRate = 40;
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
                    pidIntakeInit(999777000, 80);
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
                    pidIntakeInit(999777000, 80);
                    k++;
                }
            } else if (k == 3) {
                printf("shoot 2 ");
                printPidValues();
                if (pidIntake()) {
                    intakeRunning = true;
                    is = IntakeState::NONE;
                    pidFlywheelInit(1.5, 0.1, 9999);
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
        // auton3(autoSel_leftSide);
    } else if (autoSel_nAuton == 2) {
        // auton5(autoSel_leftSide);
    }
    stopMotorsBlock();
}
