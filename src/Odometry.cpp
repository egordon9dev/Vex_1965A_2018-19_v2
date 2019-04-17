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
void Odometry_t::setA(double newA) {
    mtxA.take(50);
    a = newA;
    mtxA.give();
}
void Odometry_t::setX(double newX) {
    mtxX.take(50);
    x = newX;
    mtxX.give();
}
void Odometry_t::setY(double newY) {
    mtxY.take(50);
    y = newY;
    mtxY.give();
}
Point Odometry_t::getPos() { return Point(getX(), getY()); }

void Odometry_t::update() {
    static double prevGyro = 0;
    odoUpdateMtx.take(50);
    double curDL = getDL(), curDR = getDR(), curDS = getDS();
    double pdl = prevDL, pdr = prevDR, pds = prevDS;
    double deltaDL = (curDL - pdl) / ticksPerInchADI, deltaDR = (curDR - pdr) / ticksPerInchADI, deltaDS = (curDS - pds) / ticksPerInchADI;
    double curGyro = getGyro();
    if (curGyro < prevGyro - PI) {
        curGyro += 2 * PI;
    } else if (curGyro > prevGyro + PI) {
        curGyro -= 2 * PI;
    }
    double deltaA = curGyro - prevGyro;  //(deltaDR - deltaDL) / (2.0 * L);
    prevGyro = curGyro;
    double chordLen = (deltaDL + deltaDR) / 2.0;
    if (!((deltaDL < -0.0001 && deltaDR > 0.0001) || (deltaDL > 0.0001 && deltaDR < -0.0001))) {  // not turning
        setX(getX() + deltaDS * cos(getA() + deltaA / 2.0 - PI / 2.0));
        setY(getY() + deltaDS * sin(getA() + deltaA / 2.0 - PI / 2.0));
    }
    setX(getX() + chordLen * cos(getA() + deltaA / 2.0));
    setY(getY() + chordLen * sin(getA() + deltaA / 2.0));
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