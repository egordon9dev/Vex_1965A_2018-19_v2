#include "Point.hpp"
#include "main.h"
#include "setup.hpp"
/*
----------------------
----    to do     ----
----------------------

- reverse kinematics
- line follower
- ultrasound
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
   ###    ##     ## ########  #######  ##    ##       ##
  ## ##   ##     ##    ##    ##     ## ###   ##     ####
 ##   ##  ##     ##    ##    ##     ## ####  ##       ##
##     ## ##     ##    ##    ##     ## ## ## ##       ##
######### ##     ##    ##    ##     ## ##  ####       ##
##     ## ##     ##    ##    ##     ## ##   ###       ##
##     ##  #######     ##     #######  ##    ##     ######
*/
/*
 void auton1(bool leftSide) {
     int sideSign = leftSide ? 1 : -1;
     // odometry.setXAxisDir(sideSign);
     // odometry.setRotationDir(sideSign);
     int i = 0;
     odometry.setA(-PI / 2);
     Point targetPos(0, 43);
     double targetAngle = -PI / 2;
     const int driveT = 500;
     IntakeState is = IntakeState::NONE;
     double arcRadius;
     int t0 = BIL;
     int prevI = 0;
     int lastT = 0;
     bool drfbPidRunning = true;
     while (1) {
         int j = 0;
         odometry.update();
         if (i == j++) {
             flywheelPid.target = 2.9;
             drfbPid.target = 1800;
             is = IntakeState::FRONT;
             if (pidDrive(targetPos, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 targetPos.x = -28 * sideSign;
                 targetPos.y = 28;
                 arcRadius = (targetPos - odometry.getPos()).mag() + 3;
                 i++;
             }
         } else if (i == j++) {
             drfbPid.target = drfbPos0 + 50;
             double distanceToTarget = (targetPos - odometry.getPos()).mag();
             if (arcRadius < distanceToTarget + 1) arcRadius = distanceToTarget + 1;
             if (pidDriveArc(targetPos, arcRadius, -sideSign, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 targetAngle = odometry.getA() + sideSign * PI / 2;
                 g_pidTurnLimit = 6000;
                 i++;
             }
         } else if (i == j++) {
             if (pidTurn(targetAngle, driveT)) {
                 g_pidTurnLimit = 12000;
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 targetPos.x = -14 * sideSign;
                 targetPos.y = 12;
                 arcRadius = (targetPos - odometry.getPos()).mag() + 1;
                 i++;
             }
         } else if (i == j++) {
             double distanceToTarget = (targetPos - odometry.getPos()).mag();
             if (arcRadius < distanceToTarget + 1) arcRadius = distanceToTarget + 1;
             if (pidDriveArc(targetPos, arcRadius, sideSign, MIL)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 targetPos.x -= 8 * sideSign;
                 i++;
             }
         } else if (i == j++) {
             if (pidDrive(targetPos, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 t0 = millis();
                 i++;
             }
         } else if (i == j++) {  // SHOOT BALL 1
             setDL(0);
             setDR(0);
             is = IntakeState::BACK;
             if (millis() - t0 > 700) {
                 is = IntakeState::NONE;
                 targetPos.x -= 14 * sideSign;
                 flywheelPid.doneTime = BIL;
                 i++;
             }
         } else if (i == j++) {
             if (pidDrive(targetPos, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 targetPos.x += 33 * sideSign;
                 i++;
             }
         } else if (i == j++) {
             if (pidDrive(targetPos, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 t0 = millis();
                 i++;
             }
         } else if (i == j++) {  // SHOOT BALL 2
             setDL(0);
             setDR(0);
             is = IntakeState::BACK;
             if (millis() - t0 > 2000) {
                 is = IntakeState::NONE;
                 targetPos.x += 11 * sideSign;
                 flywheelPid.target = 0;
                 arcRadius = (targetPos - odometry.getPos()).mag() + 5;
                 i++;
             }
         } else if (i == j++) {
             flywheelPid.target = 0;
             if (pidDrive(targetPos, driveT)) {
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 drfbPid.doneTime = BIL;
                 targetAngle -= PI / 4 * sideSign;
                 drfbPidRunning = false;
                 i++;
             }
         } else if (i == j++) {
             clawPid.target = claw180;
             bool turnDone = pidTurn(targetAngle, 0);
             bool drfbDone = getDrfb() > drfbPos2 + 200;
             if (drfbDone) {
                 drfbPid.target = getDrfb();
                 drfbPidRunning = true;
             } else {
                 setDrfb(12000);
             }
             if (turnDone && drfbDone) {
                 drfbPid.doneTime = BIL;
                 drivePid.doneTime = BIL;
                 turnPid.doneTime = BIL;
                 t0 = millis();
                 i++;
             }
         } else if (i == j++) {
             if (millis() - t0 > 2000) {
                 drfbPid.target = drfbPos2;
                 setDL(0);
                 setDR(0);
             } else {
                 if (leftSide) {
                     setDL(12000);
                     setDR(1000);
                 } else {
                     setDR(12000);
                     setDL(1000);
                 }
             }
             if (drfbPid.doneTime < millis() && millis() - t0 > 2000) {
                 drfbPid.doneTime = BIL;
                 odometry.setX(0);
                 odometry.setX(0);
                 odometry.setA(-PI / 2);
                 targetPos.x = 0;
                 targetPos.y = 15;
                 i++;
             }
         } else if (i == j++) {
             drfbPid.target = drfbPos2;
             if (pidDrive(targetPos, driveT)) {
                 setDL(0);
                 setDR(0);
                 i++;
             }
         } else {
             stopMotors();
         }
         pidClaw();
         pidFlywheel();
         if (drfbPidRunning) pidDrfb();
         setIntake(is);
         if (i != prevI) {
             for (int w = 0; w < 15; w++) cout << endl;
         }
         prevI = i;
         if (millis() - lastT > 100) {
             printDrivePidValues();
             lastT = millis();
         }
         delay(10);
     }
 }*/
/*
     ###    ##     ## ########  #######  ##    ##     #######
    ## ##   ##     ##    ##    ##     ## ###   ##    ##     ##
   ##   ##  ##     ##    ##    ##     ## ####  ##           ##
  ##     ## ##     ##    ##    ##     ## ## ## ##     #######
  ######### ##     ##    ##    ##     ## ##  ####    ##
  ##     ## ##     ##    ##    ##     ## ##   ###    ##
  ##     ##  #######     ##     #######  ##    ##    #########
*/
void auton2(bool leftSide) {
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(-4);
    double targetAngle = -PI / 2;
    const int driveT = 100;
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
    Point ptA, ptB;
    int enc0;
    while (!ctlr.get_digital(DIGITAL_B)) {
        if (i != prevI) printf("\n--------------------------------------------\n||||||||>     I has been incremented    <||||||||||\n--------------------------------------------\n\n");
        if (millis() - autonT0 > 1500000 && i != 99999) i = 12345;
        if (i != prevI) { prevITime = millis(); }
        prevI = i;
        if (millis() - prevITime > timeBetweenI) break;
        int j = 0;
        odometry.update();
        if (i == j++) {
            t0 = millis();
            pidDriveInit(Point(0, 44), driveT);
            enc0 = getDrfbEncoder();
            i++;
            k = 0;
        } else if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1");
            printf("dl %d dr %d \n", (int)getDL(), (int)getDR());
            // deploy claw
            if (k == 0) {
                setDrfb(12000);
                drfbPidRunning = false;
                if (getDrfbEncoder() - enc0 > 170) k++;
            } else {  // lower lift
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            flywheelPid.target = 2.9;
            clawPidRunning = true;
            clawPid.target = clawPos1;
            is = IntakeState::FRONT;
            if (pidDrive()) i++;
        } else if (i == j++) {
            printf("drv away cap 1");
            setDL(12000);
            setDR(12000);
            if (odometry.getY() < 40) {
                ptA = Point((leftSide ? 22 : 25.5) * sideSign, 37);
                pidFollowArcInit(Point(0, 37), ptA, leftSide ? 11.5 : 12.5, sideSign, 0);
                pidDriveArcBias(1500);
                t0 = BIL;
                i++;
            }
        } else if (i == j++) {  // arc twd cap 2
            printf("arc twd cap 2 ");
            printing = false;
            printArcData();
            printf("dl %d dr %d \n", (int)getDL(), (int)getDR());
            pidDriveArc();
            double d = (ptA - odometry.getPos()).mag();
            if (d < 3 && t0 > millis()) t0 = millis();
            if ((int)millis() - t0 > 300) {
                printing = true;
                setDL(0);
                setDR(0);
                i++;
            }
        } else if (i == j++) {
            printf("drv twd cap 2");
            setDL(12000);
            setDR(12000);
            if (odometry.getY() > 43) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // lift cap
            printf("liftCap ");
            setDL(-2000);
            setDR(-2000);
            drfbPidRunning = false;
            setDrfb(12000);
            if (getDrfb() > clawPos0 + 100) {
                drfbPidRunning = true;
                drfbPid.target = drfbPos1 + 250;
                ptB = Point(4 * sideSign, leftSide ? 36 : 24);
                pidDriveArcInit(ptA, ptB, 60, -sideSign, 0);
                DLSlew.slewRate = 40;
                DRSlew.slewRate = 40;
                i++;
            }
        } else if (i == j++) {
            printf("arc twd pipe ");
            clawPid.target = clawPos0;
            if (pidDriveArc()) {  // arc twd pipe
                DLSlew.slewRate = 120;
                DRSlew.slewRate = 120;
                t0 = BIL;
                t02 = millis();
                pidDriveArcInit(ptB, Point(27 * sideSign, leftSide ? 27 : 15), 20, -sideSign, 0);
                i++;
            }
        } else if (i == j++) {  // funnel drive
            printf("funnel drive ");
            if (millis() - t02 < 700) {
                pidDriveArc();
            } else {
                setDL(9000);
                setDR(9000);
            }
            printf("\n");
            if (millis() - t02 > 700 && !dlSaver.isFaster(0.25) && !drSaver.isFaster(0.25)) {
                if (t0 > millis()) t0 = millis();
            }
            if (millis() - t0 > 300) {
                odometry.setX(0);
                odometry.setY(0);
                odometry.setA(leftSide ? 0 : PI);
                dlSaver.reset();
                drSaver.reset();
                ptA = Point(-sideSign * 7, 0);
                pidDriveInit(ptA, driveT);
                i++;
            }
        } else if (i == j++) {
            printf("align for cap place ");
            printPidValues();
            printf("\n");
            if (pidDrive()) {
                drfbPid.target = drfbPos0;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            printf("place cap ");
            printPidValues();
            printf("dl %d dr %d \n", (int)getDL(), (int)getDR());
            drfbPidRunning = false;
            setDL(0);
            setDR(0);
            setDrfb(-12000);
            if (getDrfb() < drfbPos1 + 50 || millis() - t0 > 800) {
                drfbPid.target = drfbPos1;
                drfbPidRunning = true;
                t0 = millis();
                t02 = BIL;
                ptB = Point(-sideSign * 22, 5);
                pidDriveArcInit(ptA, ptB, 100, -sideSign, driveT);
                i++;
            }
        } else if (i == j++) {
            printing = false;
            printf("arc to shoot pos ");
            printArcData();
            printf("dl %d dr %d \n", (int)getDL(), (int)getDR());
            if (millis() - t0 > 300) drfbPid.target = drfbPos0;
            pidDriveArc();
            if ((odometry.getPos() - ptB).mag() < 3 && t02 > millis()) t02 = millis();
            if (millis() - t02 > 0) {
                printing = true;
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // shoot
            printf("shoot");
            setDL(0);
            setDR(0);
            is = IntakeState::BACK;
            if (millis() - t0 > 500) {
                t0 = millis();
                flywheelPid.target = 3.0;
                i++;
            }
        } else if (i == j++) {
            is = IntakeState::NONE;
            if (millis() - autonT0 > 14300 || millis() - t0 > 1000) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            is = IntakeState::BACK;
            if (millis() - t0 > 1000) { i++; }
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
    stopMotors();
}
/*
   ###    ##     ## ########  #######  ##    ##     #######
  ## ##   ##     ##    ##    ##     ## ###   ##    ##     ##
 ##   ##  ##     ##    ##    ##     ## ####  ##           ##
##     ## ##     ##    ##    ##     ## ## ## ##     #######
######### ##     ##    ##    ##     ## ##  ####           ##
##     ## ##     ##    ##    ##     ## ##   ###    ##     ##
##     ##  #######     ##     #######  ##    ##     #######
*/
void auton3(bool leftSide) {
    int sideSign = leftSide ? 1 : -1;
    int i = 0;
    int k = 0;
    odometry.setA(-PI / 2);
    odometry.setX(0);
    odometry.setY(-4);
    double targetAngle = -PI / 2;
    const int driveT = 200;
    IntakeState is = IntakeState::NONE;
    double arcRadius;
    int t0 = BIL;
    int prevI = 0;
    int lastT = 0;
    bool drfbPidRunning = false, clawPidRunning = false;
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
        if (millis() - prevITime > timeBetweenI) break;
        int j = 0;
        odometry.update();
        if (i == j++) {
            t0 = millis();
            ptB = Point(0, 46);
            pidDriveInit(ptB, driveT);
            enc0 = getDrfbEncoder();
            flywheelPid.target = 3.0;
            i++;
            k = 0;
        } else if (i == j++) {  // grab ball from under cap 1
            printf("drv twd cap 1");
            // deploy claw
            if (k == 0) {
                setDrfb(12000);
                drfbPidRunning = false;
                if (getDrfbEncoder() - enc0 > 170) k++;
            } else {  // lower lift
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
            }
            clawPidRunning = true;
            clawPid.target = clawPos1;
            is = IntakeState::FRONT;
            if (pidDrive()) {
                drfbPidRunning = true;
                drfbPid.target = drfbPos0;
                ptA = Point(11 * sideSign, 10);
                if (!leftSide) {
                    ptA.y += 4;
                    ptA.x -= 2;
                }
                pidDriveArcInit(ptB, ptA, 65, sideSign, driveT);
                i++;
            }
        } else if (i == j++) {  // drive back
            if (pidDriveArc()) {
                is = IntakeState::NONE;
                targetAngle += sideSign * PI * 0.538;
                pidTurnInit(targetAngle, driveT);
                i++;
            }
        } else if (i == j++) {  // turn to face flags
            if (pidTurn()) {
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // shoot
            setDL(0);
            setDR(0);
            is = IntakeState::BACK;
            if (millis() - t0 > 500) {
                pidDriveArcInit(ptA, Point(-20 * sideSign, -1), 40, -sideSign, driveT);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {
            is = IntakeState::NONE;
            if (pidDriveArc()) {
                i++;
                t0 = millis();
            }
        } else if (i == j++) {
            is = IntakeState::BACK;
            setDL(0);
            setDR(0);
            if (millis() - t0 > 1400) {
                pidDriveInit(Point(-48 * sideSign, -3), driveT);
                t0 = millis();
                i++;
            }
        } else if (i == j++) {  // knock bottom flag
            is = IntakeState::FRONT;
            bool stall = millis() - t0 > 200 && !dlSaver.isFaster(0.1) && !drSaver.isFaster(0.1) && (dlSaver.isPwr(0.25) || drSaver.isPwr(0.25));
            if (pidDrive() || stall) { i++; }
        } else if (i == j++) {
            setDL(12000);
            setDR(12000);
            if (millis() - t0 > 600) i++;
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
    stopMotors();
}
void autonomous() {
    setup();
    auton3(false);
    stopMotors();
}
