#include "main.h"
#include "pid.hpp"
#include "setup.hpp"
pros::Mutex mtxX, mtxY, mtxA;
pros::Mutex odoUpdateMtx;
Odometry_t odometry(6.982698, 1.0);
Odometry_t::Odometry_t(double L, double perpL) {
    this->L = L;
    this->perpL = perpL;
    this->a = PI / 2;
    this->x = 0;
    this->y = 0;
    this->prevDL = this->prevDR = this->prevDS = 0.0;
}
double Odometry_t::getX() {
    mtxX.take(50);
    double d = x;
    mtxX.give();
    return d;
}
double Odometry_t::getY() {
    mtxY.take(50);
    double d = y;
    mtxY.give();
    return d;
}
double Odometry_t::getA() {
    mtxA.take(50);
    double d = a;
    mtxA.give();
    return d;
}
void Odometry_t::setA(double a) {
    mtxA.take(50);
    this->a = a;
    mtxA.give();
}
void Odometry_t::setX(double x) {
    mtxX.take(50);
    this->x = x;
    mtxX.give();
}
void Odometry_t::setY(double y) {
    mtxY.take(50);
    this->y = y;
    mtxY.give();
}
Point Odometry_t::getPos() { return Point(getX(), getY()); }

void Odometry_t::update() {
    odoUpdateMtx.take(50);
    double curDL = getDL(), curDR = getDR(), curDS = getDS();
    double pdl = prevDL, pdr = prevDR, pds = prevDS;
    double deltaDL = (curDL - pdl) / ticksPerInchADI, deltaDR = (curDR - pdr) / ticksPerInchADI, deltaDS = (curDS - pds) / ticksPerInchADI;
    double deltaA = (deltaDR - deltaDL) / (2.0 * L);
    double chordLen = (deltaDL + deltaDR) / 2.0;
    if (!((deltaDL < -0.0001 && deltaDR > 0.0001) || (deltaDL > 0.0001 && deltaDR < -0.0001))) {  // not turning
        setX(getX() + deltaDS * cos(a + deltaA / 2.0 - PI / 2.0));
        setY(getY() + deltaDS * sin(a + deltaA / 2.0 - PI / 2.0));
    }
    setX(getX() + chordLen * cos(a + deltaA / 2.0));
    setY(getY() + chordLen * sin(a + deltaA / 2.0));
    setA(getA() + deltaA);
    prevDL = curDL;
    prevDR = curDR;
    prevDS = curDS;
    odoUpdateMtx.give();
}

void zeroDriveEncs();
void Odometry_t::reset() {
    odoUpdateMtx.take(50);
    zeroDriveEncs();
    prevDL = prevDR = prevDS = 0.0;
    odoUpdateMtx.give();
}