#ifndef SETUP_H
#define SETUP_H
#include "MotorSaver.hpp"
#include "main.h"
#include "pid.hpp"
#define TICKS_TO_DEG 0.4
extern pros::Motor mtr5, mtr6, mtr7, mtr8, mtr9, mtr10, mtr13, mtr12;
extern MotorSaver dlSaver, drSaver, drfbSaver, clawSaver, flySaver, intakeSaver;
extern pros::Controller ctlr;
extern pros::ADIPotentiometer* drfbPot;
extern pros::ADILineSensor* ballSensL;
extern pros::ADILineSensor* ballSensR;
extern pros::ADIEncoder* perpindicularWheelEnc;
extern const int drfbMinPos, drfbMaxPos, drfbPos0, drfbPos1, drfbPos2, drfbMinClaw0, drfbMaxClaw0, drfbMinClaw1, claw180, clawPos0, clawPos1, drfb18Max;
extern const int ctlrIdxLeft, ctlrIdxUp, ctlrIdxRight, ctlrIdxDown, ctlrIdxY, ctlrIdxX, ctlrIdxA, ctlrIdxB, ctlrIdxL1, ctlrIdxL2, ctlrIdxR1, ctlrIdxR2;
extern const int BIL, MIL;
extern const int dblClickTime;
extern const double PI;
extern const double ticksPerInch;
extern const double ticksPerInchADI;
enum class IntakeState { FRONT, BACK, ALTERNATE, NONE };
int clamp(int n, int min, int max);
double clamp(double n, double min, double max);

//------- Misc ----------
// returns prevClicks, curClicks, DblClicks
bool** getAllClicks();
void printAllClicks(int line, bool** allClicks);
void printPidValues();
void printState();
void stopMotors();
Point polarToRect(double mag, double angle);
int millis();

// -------- Drive --------
void setDR(int n);
void setDL(int n);
double getDR();
double getDL();
double getDS();
int getDLVoltage();
int getDRVoltage();
void printDrivePidValues();
void printDriveEncoders();
void runMotorTest();

//----------- Intake ------
void setIntake(IntakeState is);
int getBallSensL();
int getBallSensR();
bool isBallIn();
IntakeState getISLoad();

//----------- DRFB functions ---------
void setDrfb(int n);
void setDrfbParams(bool auton);
int getDrfb();
int getDrfbEncoder();
bool pidDrfb(double pos, int wait);
void pidDrfb();
//---------- Claw functions --------
void setClawPosition(double pos);
void setClaw(int n);
double getClaw();
int getClawVoltage();
bool pidClaw(double a, int wait);
void pidClaw();
//--------- Flywheel functions --------
void setFlywheel(int n);
double getFlywheel();
int getFlywheelVoltage();
bool pidFlywheel();
bool pidFlywheel(int pwr0, double speed);
bool pidFlywheel(int pwr0, double speed, int wait);
void setup();

void testDriveMtrs();
#endif
