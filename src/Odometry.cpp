#include "main.h"
#include "pid.hpp"
#include "setup.hpp"
Odometry_t odometry(6.982698, 1.0);
Odometry_t bentOdo(6.982698, 1.0);
Odometry_t::Odometry_t(double L, double perpL) {
    this->L = L;
    this->perpL = perpL;
    this->a = PI / 2;
    this->x = 0;
    this->y = 0;
    this->prevDL = this->prevDR = this->prevDS = 0.0;
    this->prevGyro = 0;
    this->scaleL = 1.0;
    this->scaleR = 1.0;
}
// bending space WooOOoooHoo
void Odometry_t::setScaleL(double scL) {
    mtxScaleL.take(50);
    scaleL = scL;
    mtxScaleL.give();
}
void Odometry_t::setScaleR(double scR) {
    mtxScaleR.take(50);
    scaleR = scR;
    mtxScaleR.give();
}
void Odometry_t::setScales(double scL, double scR) {
    setScaleL(scL);
    setScaleR(scR);
}
double Odometry_t::getScaleL() {
    mtxScaleL.take(50);
    double d = scaleL;
    mtxScaleL.give();
    return d;
}
double Odometry_t::getScaleR() {
    mtxScaleR.take(50);
    double d = scaleR;
    mtxScaleR.give();
    return d;
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
    odoUpdateMtx.take(50);
    double curDL = getDL(), curDR = getDR(), curDS = getDS();
    double pdl = prevDL, pdr = prevDR, pds = prevDS;
    double deltaDL = (curDL - pdl) * getScaleL() / ticksPerInchADI, deltaDR = (curDR - pdr) * getScaleR() / ticksPerInchADI, deltaDS = (curDS - pds) / ticksPerInchADI;
    double curGyro = getGyro();
    if (curGyro < prevGyro - PI) {
        curGyro += 2 * PI;
    } else if (curGyro > prevGyro + PI) {
        curGyro -= 2 * PI;
    }
    double deltaA = 0.0;
    if (fabs(getScaleL() - getScaleR()) < 0.001) {
        deltaA = curGyro - prevGyro;
    } else {
        deltaA = (deltaDR - deltaDL) / (2.0 * L);
    }
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

void Odometry_t::reset() {
    odoUpdateMtx.take(50);
    prevDL = getDL();
    prevDR = getDR();
    prevDS = getDS();
    odoUpdateMtx.give();
}