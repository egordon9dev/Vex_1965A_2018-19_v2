#ifndef SETUP_H
#define SETUP_H
#include "MotorSaver.hpp"
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
extern MotorSaver dlSaver, drSaver, drfbSaver, clawSaver, flySaver, intakeSaver;
extern pros::Controller ctlr, ctlr2;
extern pros::ADILineSensor* ballSensL;
extern pros::ADILineSensor* ballSensR;
// extern pros::ADIEncoder* perpindicularWheelEnc;
extern pros::ADIEncoder* DLEnc;
extern pros::ADIEncoder* DREnc;
extern pros::Vision* vision;
extern pros::ADIGyro* gyro;
extern const int driveTurnLim;
extern const int drfbMinPos, drfbMaxPos, drfbPos0, drfbPos1, drfbPos2, drfbPos1Plus, drfbPos2Plus, drfbMinClaw0, drfbMaxClaw0, drfbMinClaw1, drfb18Max, drfbPosCloseIntake, drfbPosScrape, drfbPosAboveScrape;
extern const int drfbHoldPwr;
extern double drfbIMEBias;
extern const int claw0, claw180;
extern const int intakeOneShotTicks, intakeOneShotTicksTop;
extern double sShotSpeed, fw_a4_middleFlag, fw_a4_sideFlag;
extern const int ctlrIdxLeft, ctlrIdxUp, ctlrIdxRight, ctlrIdxDown, ctlrIdxY, ctlrIdxX, ctlrIdxA, ctlrIdxB, ctlrIdxL1, ctlrIdxL2, ctlrIdxR1, ctlrIdxR2;
extern int drfbPidBias;
extern const int BIL, MIL;
extern const int dblClickTime;
extern const double PI; /*
 extern const double ticksPerInch;*/
extern const double ticksPerInchADI;
extern int driveLim;
extern int drfbFullRangePowerLimit;
enum class IntakeState { FRONT, FRONT_HOLD, BACK, BACK_HOLD, BACK_SLOW, ALTERNATE, NONE };

extern bool g_isAuton;
int clamp(int n, int min, int max);
double clamp(double n, double min, double max);

extern bool autoSel_leftSide;
extern int autoSel_nAuton;
void setup();
void morningRoutine();
void autoSel_update();

//------- Misc ----------
// returns prevClicks, curClicks, DblClicks
bool** getAllClicks();
void printAllClicks(int line, bool** allClicks);
void printPidValues();
void printPidSweep();
void stopMotors();
void stopMotorsBlock();
Point polarToRect(double mag, double angle);
int millis();

// -------- Drive --------
void opctlDrive(int driveDir);
void setDR(int n);
void setDL(int n);
double getDR();
double getDL();
double getDS();
double getDLVel();
double getDRVel();
double getDriveVel();
double getGyro();
int getDLVoltage();
int getDRVoltage();
void printDrivePidValues();
void printDriveEncoders();
void runMotorTest();
void setDriveSlew(bool auton);
void setMaxAErr(double mae);
void driveToCap(bool red, int pwr, double offset);

//----------- Intake ------
void setIntake(IntakeState is);
void setIntake(int n);
int getBallSensTop();
int getBallSensBtm();
bool isTopBallIn();
bool isBtmBallIn();
double getIntakePos();
double getIntakeVel();
int getIntakeVoltage();
void pidIntakeInit(double target, int wait);
bool pidIntake();

//----------- DRFB functions ---------
void trimDrfb(int trim);
void setDrfb(int n);
void setDrfbDull(int n);
void setDrfbDumb(int n);
void setDrfbParams(bool auton);
double getDrfb();
double getDrfbVel();
int getDrfbCurrent();
int getDrfbVoltage();
bool pidDrfb(double pos, int wait);
void pidDrfb();
//---------- Claw functions --------
void setClawPosition(double pos);
void setClaw(int n);
double getClaw();
double getClawVel();
int getClawVoltage();
bool pidClaw(double a, int wait);
void pidClaw();
//--------- Flywheel functions --------
void setFlywheel(int n);
double getFlywheel();
int getFlywheelVoltage();
int getFlywheelMeasuredVoltage();
bool pidFlywheel();
void pidFlywheelInit(double speed, double pidZone, int wait);
bool isPidFlywheelDone();

double getCapX(bool red);
void testDriveMtrs();
#endif
