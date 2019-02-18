#ifndef SETUP_H
#define SETUP_H
#include "MotorSaver.hpp"
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
extern pros::Motor mtr1, mtr2, mtr3, mtr4, mtr5, mtr6, mtr7, mtr8;
extern MotorSaver dlSaver, drSaver, drfbSaver, clawSaver, flySaver, intakeSaver;
extern pros::Controller ctlr, ctlr2;
extern pros::ADILineSensor* ballSensL;
extern pros::ADILineSensor* ballSensR;
extern pros::ADIEncoder* perpindicularWheelEnc;
extern pros::ADIEncoder* DLEnc;
extern pros::ADIEncoder* DREnc;
extern const int drfbMinPos, drfbMaxPos, drfbPos0, drfbPos1, drfbPos2, drfbPos1Plus, drfbPos2Plus, drfbMinClaw0, drfbMaxClaw0, drfbMinClaw1, drfb18Max;
extern double drfbIMEBias;
extern const int claw180;
extern const int intakeShootTicks;
extern const double dShotSpeed1, dShotSpeed2;
extern const int ctlrIdxLeft, ctlrIdxUp, ctlrIdxRight, ctlrIdxDown, ctlrIdxY, ctlrIdxX, ctlrIdxA, ctlrIdxB, ctlrIdxL1, ctlrIdxL2, ctlrIdxR1, ctlrIdxR2;
extern int drfbPidBias;
extern const int BIL, MIL;
extern const int dblClickTime;
extern const double PI; /*
 extern const double ticksPerInch;*/
extern const double ticksPerInchADI;
extern double DLEncBias, DREncBias, DSEncBias;
extern int driveLim;
extern int clawPowerLimit;
extern int drfbFullRangePowerLimit;
enum class IntakeState { FRONT, FRONT_SLOW, BACK, BACK_SLOW, ALTERNATE, NONE };
int clamp(int n, int min, int max);
double clamp(double n, double min, double max);

void setup();
void morningRoutine();

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
int getDLVoltage();
int getDRVoltage();
void printDrivePidValues();
void printDriveEncoders();
void runMotorTest();
void setDriveSlew(bool auton);
void zeroDriveEncs();

//----------- Intake ------
void setIntake(IntakeState is);
void setIntake(int n);
int getBallSensL();
int getBallSensR();
bool isBallIn();
IntakeState getISLoad();
double getIntakePos();
int getIntakeVoltage();
void pidIntakeInit(double target, int wait);
bool pidIntake();

//----------- DRFB functions ---------
void trimDrfb(int trim);
void setDrfb(int n);
void setDrfbDumb(int n);
void setDrfbParams(bool auton);
double getDrfb();
int getDrfbCurrent();
bool pidDrfb(double pos, int wait);
void pidDrfb();
//---------- Claw functions --------
void setClawPosition(double pos);
void setClaw(int n);
void setClaw(int n, bool limit);
double getClaw();
int getClawVoltage();
bool pidClaw(double a, int wait);
void pidClaw();
//--------- Flywheel functions --------
void setFlywheel(int n);
double getFlywheel();
int getFlywheelVoltage();
bool pidFlywheel();
void pidFlywheelInit(double speed, int wait);
bool isPidFlywheelDone();

void testDriveMtrs();
#endif
