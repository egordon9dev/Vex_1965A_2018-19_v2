#include "pid.hpp"
#include "main.h"
#include "setup.hpp"
using std::cout;
Pid_t flywheelPid, clawPid, drfbPid, DLPid, DRPid, drivePid, turnPid, curvePid;
Slew_t flywheelSlew, drfbSlew, DLSlew, DRSlew, clawSlew, intakeSlew;
Odometry_t odometry(6.982698, 1.0);
/*
 ########  #### ########           #######  ########   #######
 ##     ##  ##  ##     ##         ##     ## ##     ## ##     ##
 ##     ##  ##  ##     ##         ##     ## ##     ## ##     ##
 ########   ##  ##     ## ####    ##     ## ##     ## ##     ## ####
 ##         ##  ##     ## ####    ##     ## ##     ## ##     ## ####
 ##         ##  ##     ##  ##     ##     ## ##     ## ##     ##  ##
 ##        #### ########  ##       #######  ########   #######  ##

  ######  ##       ######## ##      ##
 ##    ## ##       ##       ##  ##  ##
 ##       ##       ##       ##  ##  ##
  ######  ##       ######   ##  ##  ##
       ## ##       ##       ##  ##  ##
 ##    ## ##       ##       ##  ##  ##
  ######  ######## ########  ###  ###
*/
Slew_t::Slew_t() {
    slewRate = 100.0;
    output = 0;
    prevTime = millis();
}
Pid_t::Pid_t() {
    doneTime = BIL;
    DONE_ZONE = 10;
    maxIntegral = 9999999;
    dInactiveZone = iActiveZone = target = prevSensVal = sensVal = prevErr = errTot = unwind = deriv = prop = kp = ki = kd = 0.0;
    derivativeUpdateInterval = 15;
    prevTime = prevDUpdateTime = 0;
}
Odometry_t::Odometry_t(double L, double perpL) {
    this->L = L;
    this->perpL = perpL;
    this->a = PI / 2;
    this->x = this->y = this->prevDL = this->prevDR = this->prevDS = 0.0;
}
double Odometry_t::getX() { return x; }
double Odometry_t::getY() { return y; }
double Odometry_t::getA() { return a; }
void Odometry_t::setA(double a) { this->a = a; }
void Odometry_t::setX(double x) { this->x = x; }
void Odometry_t::setY(double y) { this->y = y; }
Point Odometry_t::getPos() {
    Point p(x, y);
    return p;
}

void Odometry_t::update() {
    double curDL = getDL(), curDR = getDR(), curDS = getDS();
    double deltaDL = (curDL - prevDL) / ticksPerInchADI, deltaDR = (curDR - prevDR) / ticksPerInchADI, deltaDS = (curDS - prevDS) / ticksPerInchADI;
    double deltaA = (deltaDR - deltaDL) / (2.0 * L);
    double chordLen = (deltaDL + deltaDR) / 2.0;
    if (!((deltaDL < -0.0001 && deltaDR > 0.0001) || (deltaDL > 0.0001 && deltaDR < -0.0001))) {  // not turning
        x += deltaDS * cos(a + deltaA / 2.0 - PI / 2.0);
        y += deltaDS * sin(a + deltaA / 2.0 - PI / 2.0);
    }
    x += chordLen * cos(a + deltaA / 2.0);
    y += chordLen * sin(a + deltaA / 2.0);
    a += deltaA;
    printf("(%.1f, %.1f, %.2f) (%d %d %d) ", x, y, a, (int)curDL, (int)curDR, (int)curDS);
    prevDL = curDL;
    prevDR = curDR;
    prevDS = curDS;
}

void Odometry_t::reset() {
    zeroDriveEncs();
    prevDL = prevDR = prevDS = 0.0;
}

/*
  in: input voltage
*/
double Slew_t::update(double in) {
    int dt = millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = millis();
    double maxIncrease = slewRate * dt;
    double outputRate = (double)(in - output) / (double)dt;
    if (fabs(outputRate) < slewRate) {
        output = in;
    } else if (outputRate > 0) {
        output += maxIncrease;
    } else {
        output -= maxIncrease;
    }
    return output;
}
// proportional + integral + derivative control feedback
double Pid_t::update() {
    int dt = millis() - prevTime;
    if (dt > 1000) dt = 0;
    prevTime = millis();
    // PROPORTIONAL
    double err = target - sensVal;
    double p = err * kp;
    // DERIVATIVE
    double d = deriv;  // set d to old derivative
    double derivativeDt = millis() - prevDUpdateTime;
    if (derivativeDt == 0) derivativeDt = 0.001;
    if (derivativeDt > 1000) {
        prevSensVal = sensVal;
        prevDUpdateTime = millis();
        d = 0;
    } else if (derivativeDt >= derivativeUpdateInterval) {
        d = ((prevSensVal - sensVal) * kd) / derivativeDt;
        prevDUpdateTime = millis();
        deriv = d;  // save new derivative
        prevSensVal = sensVal;
    }
    if (fabs(err) < dInactiveZone) d = 0;
    // INTEGRAL
    errTot += err * dt;
    if (fabs(err) > iActiveZone) errTot = 0;
    // if (fabs((prevSensVal - sensVal) / derivativeDt) > 0.02) {
    double maxErrTot = ((ki != 0.0) ? (maxIntegral / ki) : 999999999);
    if (errTot > maxErrTot) errTot = maxErrTot;
    if (errTot < -maxErrTot) errTot = -maxErrTot;
    //}
    if (err > 0.0 && errTot < 0.0 || err < 0.0 && errTot > 0.0 || fabs(err) < 0.001) {
        if (fabs(err) - unwind > -0.001) {
            errTot = 0.0;
            // printf("UNWIND\n");
        }
    }
    if (fabs(unwind) < 0.001 && fabs(err) < 0.001) {
        errTot = 0.0;
        // printf("UNWIND\n");
    }
    double i = errTot * ki;
    // done zone
    if (fabs(err) <= DONE_ZONE && doneTime > millis()) {
        doneTime = millis();
        // printf("DONE\n");
    }
    // derivative action: slowing down
    /*if (fabs(d) > (fabs(p)) * 20.0) {
          errTot = 0.0;
      }*/
    prevErr = err;
    // OUTPUT
    prop = p;
    // printf("<%d, %d, %d> ", (int)p, (int)i, (int)d);
    return p + i + d;
}

/*
 ######## ##     ## ########  ##    ##
    ##    ##     ## ##     ## ###   ##
    ##    ##     ## ##     ## ####  ##
    ##    ##     ## ########  ## ## ##
    ##    ##     ## ##   ##   ##  ####
    ##    ##     ## ##    ##  ##   ###
    ##     #######  ##     ## ##    ##
*/
int g_pidTurnLimit = 12000;
namespace turnData {
double angle;
int wait;
int doneT;
void init(double a, int w) {
    angle = a;
    wait = w;
    doneT = BIL;
}
}  // namespace turnData
void pidTurnInit(const double angle, const int wait) {
    turnData::init(angle, wait);
    turnPid.doneTime = BIL;
}
bool pidTurn() {
    using turnData::angle;
    using turnData::doneT;
    using turnData::wait;
    turnPid.sensVal = odometry.getA();
    turnPid.target = angle;
    int pwr = clamp((int)turnPid.update(), -g_pidTurnLimit, g_pidTurnLimit);
    setDL(-pwr);
    setDR(pwr);
    static double prevA = turnPid.sensVal;
    // printf("%f %f\n", fabs(turnPid.sensVal - turnPid.target), fabs(turnPid.sensVal - prevA));
    if (fabs(turnPid.sensVal - turnPid.target) < 0.1 && fabs(turnPid.sensVal - prevA) < 0.001) {
        if (doneT > millis()) doneT = millis();
    } else {
        doneT = BIL;
    }
    prevA = turnPid.sensVal;
    return doneT + wait < millis();
}
double getFaceA(const Point& p, bool flip) {
    Point delta = p - odometry.getPos();
    double curA = odometry.getA();
    if (delta.mag() == 0.0) return curA;
    Point oVector = polarToRect(1, curA);
    double aErr = delta.angleBetween(oVector);
    if (flip) aErr = PI - aErr;
    if (!flip && oVector < delta) aErr *= -1;
    if (flip && oVector > delta) aErr *= -1;
    return curA - aErr;
}
void pidFaceInit(const Point& p, bool flip, const int wait) { pidTurnInit(getFaceA(p, flip), wait); }
bool pidFace() { return pidTurn(); }
bool bangTurn(double a) {
    static double prevA = -BIL;
    static int dir = 1;
    double err = odometry.getA() - a;
    if (fabs(a - prevA) > 0.001) dir = (err > 0 ? -1 : 1);
    prevA = a;
    int output = err > 0 ? -12000 : 12000;
    setDL(-output);
    setDR(output);
    return err * dir > 0;
}
namespace sweep {
int dl0, dr0;
double tL, tR;
int wait;
void init(double tL, double tR, int wait) {
    this->tL = tL;
    this->tR = tR;
    this->wait = wait;
    DLPid.doneTime = DRPid.doneTime = BIL;
    dl0 = getDL();
    dr0 = getDR();
}
}  // namespace sweep
bool pidSweepInit(double tL, double tR, int wait) { sweep::init(tL, tR, wait); }
bool pidSweep() {
    using sweep::tL;
    using sweep::tR;
    using sweep::wait;
    DLPid.sensVal = getDL() - sweep::dl0;
    DRPid.sensVal = getDR() - sweep::dr0;
    DLPid.target = tL * ticksPerInchADI;
    DRPid.target = tR * ticksPerInchADI;
    if (DLPid.target == 0.0) DLPid.target = 0.000001;
    if (DRPid.target == 0.0) DRPid.target = 0.000001;
    double powerL = clamp(DLPid.update(), -12000.0, 12000.0);
    double powerR = clamp(DRPid.update(), -12000.0, 12000.0);
    int curve = 0;
    curvePid.sensVal = DRPid.sensVal - DLPid.sensVal * ((double)tR / (double)tL);
    curvePid.target = 0.0;
    // curve is scaled down as the motion progresses
    curve = curvePid.update() * 0.5 * (fabs((DL_pid.target - DL_pid.sensVal) / DL_pid.target) + fabs((DR_pid.target - DR_pid.sensVal) / DR_pid.target));
    double curveInfluence = 0.75;
    if (fabs(powerR) > fabs(powerL)) {
        curve = clamp(curve, -fabs(powerL) * curveInfluence, fabs(powerL) * curveInfluence);
    } else {
        curve = clamp(curve, -fabs(powerR) * curveInfluence, fabs(powerR) * curveInfluence);
    }
    powerL -= curve;
    powerR += curve;
    setDL(powerL);
    setDR(powerR);
    if (DLPid.doneTime + wait < millis() && DRPid.doneTime + wait < millis()) return true;
    return false;
}
/*
    ###    ########   ######
   ## ##   ##     ## ##    ##
  ##   ##  ##     ## ##
 ##     ## ########  ##
 ######### ##   ##   ##
 ##     ## ##    ##  ##    ##
 ##     ## ##     ##  ######
*/
namespace arcData {
const int pwrLim1 = 7500, pwrLim2 = pwrLim1 + 1000;
int doneT;
Point center;
Point _target, _start;
double _rMag;
int _rotationDirection;
int wait;
int bias;
bool followArc;
void init(Point start, Point target, double rMag, int rotationDirection) {
    target.noZeroes();
    start.noZeroes();
    doneT = BIL;
    _start = start;
    _target = target;
    _rotationDirection = rotationDirection;
    _rMag = rMag;
    bias = 0;
    followArc = false;

    Point deltaPos = target - start;
    Point midPt((start.x + target.x) / 2.0, (start.y + target.y) / 2.0);
    double altAngle = atan2(deltaPos.y, deltaPos.x) + (PI / 2) * rotationDirection;
    double altMag = sqrt(clamp(pow(rMag, 2) - pow(deltaPos.mag() / 2, 2), 0.0, 999999999.9));
    center = midPt + polarToRect(altMag, altAngle);
}
// estimate the distance remaining for the drive
// assumes you would never arc more than about 7*PI / 4
double getArcPos() {
    Point pos = odometry.getPos() - center;
    Point tgt = _target - center;
    Point st = _start - center;
    double a = acos(clamp((pos * tgt) / (pos.mag() * tgt.mag()), -1.0, 1.0));
    if (_rotationDirection == 1) {
        if (pos < st && a > PI / 2) a = 2 * PI - a;
    } else {
        if (pos > st && a > PI / 2) a = 2 * PI - a;
    }
    return a * pos.mag();
}
}  // namespace arcData

void printArcData() {
    printf("%.1f DL%d DR%d drive %3.1f/%3.1f curve %2.3f/%2.3f R %.1f/%.1f x %3.1f/%3.1f y %3.1f/%3.1f a %.1f\n", millis() / 1000.0, (int)(getDLVoltage() / 100 + 0.5), (int)(getDRVoltage() / 100 + 0.5), drivePid.sensVal, drivePid.target, curvePid.sensVal, curvePid.target, (odometry.getPos() - arcData::center).mag(), arcData::_rMag, odometry.getX(), arcData::_target.x, odometry.getY(), arcData::_target.y, odometry.getA());
    std::cout << std::endl;
}
void pidDriveArcInit(Point start, Point target, double rMag, int rotationDirection, int wait) {
    drivePid.doneTime = BIL;
    turnPid.doneTime = BIL;
    curvePid.doneTime = BIL;
    arcData::init(start, target, rMag, rotationDirection);
    arcData::wait = wait;
}

void pidFollowArcInit(Point start, Point target, double rMag, int rotationDirection, int wait) {
    pidDriveArcInit(start, target, rMag, rotationDirection, wait);
    arcData::followArc = true;
}
void pidDriveArcBias(int b) { arcData::bias = b; }
bool pidDriveArc() {
    using arcData::_rMag;
    using arcData::_rotationDirection;
    using arcData::_start;
    using arcData::_target;
    using arcData::center;
    using arcData::doneT;
    using arcData::pwrLim1;
    using arcData::pwrLim2;
    using arcData::wait;
    Point pos = odometry.getPos();
    double arcPos = arcData::getArcPos();

    Point rVector = center - pos;
    Point targetVector = rVector.rotate(-_rotationDirection);
    Point orientationVector(cos(odometry.getA()), sin(odometry.getA()));
    double x = (targetVector * orientationVector) / (targetVector.mag() * orientationVector.mag());
    double errAngle = acos(clamp(x, -1.0, 1.0));
    double errRadius = rVector.mag() - _rMag;
    // allow for driving backwards and allow for negative errAngle values
    int driveDir = 1;
    if (errAngle > PI / 2) {
        driveDir = -1;
        errAngle = PI - errAngle;
    }
    if (orientationVector < targetVector) errAngle *= -driveDir;
    if (orientationVector > targetVector) errAngle *= driveDir;
    // printf("center: %.1f,%.1f pos: %.1f,%.1ftarget:%.1f,%.1f\n", center.x, center.y, pos.x, pos.y, arcData::_target.x, arcData::_target.y);
    // error correction
    curvePid.sensVal = errAngle /*- clamp(errRadius * _rotationDirection * (PI / 6), -PI / 3, PI / 3)*/;
    curvePid.target = 0;
    Point rVec = pos - center, tgt = _target - center;
    if (_rotationDirection == 1) {
        if (rVec > tgt) arcPos *= -1;
    } else {
        if (rVec < tgt) arcPos *= -1;
    }
    if (fabs(arcPos) > rVec.mag() * PI) arcPos *= -1;
    drivePid.sensVal = arcPos;
    drivePid.target = 0;
    double drivePwr = -drivePid.update();
    if (arcData::followArc) drivePwr = pwrLim1;
    int turnPwr = clamp((int)curvePid.update(), -pwrLim1, pwrLim1);
    double pwrFactor = 1;  // clamp(1.0 / (1.0 + fabs(errRadius) / 2.0) * 1.0 / (1.0 + fabs(errAngle) * 6), 0.5, 1.0);
    double curveFac = clamp(2.0 / (1.0 + exp(-_rMag / 7.0)) - 1.0, 0.001, 1.0);
    double dlOut = clamp(drivePwr * pwrFactor * driveDir, (double)-pwrLim1, (double)pwrLim1);
    double drOut = clamp(drivePwr * pwrFactor * driveDir, (double)-pwrLim1, (double)pwrLim1);
    if (_rotationDirection * driveDir == -1) {
        dlOut *= 1.0 / curveFac;
        drOut *= curveFac;
    } else {
        dlOut *= curveFac;
        drOut *= 1.0 / curveFac;
    }
    dlOut -= _rotationDirection * driveDir * arcData::bias;
    drOut += _rotationDirection * driveDir * arcData::bias;
    setDL(clamp((int)dlOut, -pwrLim2, pwrLim2) - turnPwr);
    setDR(clamp((int)drOut, -pwrLim2, pwrLim2) + turnPwr);
    static Point prevPos(0, 0);

    if (fabs(drivePid.sensVal) < 2 && (pos - prevPos).mag() < 0.01) {
        if (doneT > millis()) doneT = millis();
    } else {
        doneT = BIL;
    }
    prevPos = pos;
    return doneT + wait < millis();
}

/*
 ########  #### ########     ########  ########  #### ##     ## ########
 ##     ##  ##  ##     ##    ##     ## ##     ##  ##  ##     ## ##
 ##     ##  ##  ##     ##    ##     ## ##     ##  ##  ##     ## ##
 ########   ##  ##     ##    ##     ## ########   ##  ##     ## ######
 ##         ##  ##     ##    ##     ## ##   ##    ##   ##   ##  ##
 ##         ##  ##     ##    ##     ## ##    ##   ##    ## ##   ##
 ##        #### ########     ########  ##     ## ####    ###    ########
*/
Point g_target;
namespace driveData {
Point start, target;
int wait;
int doneT;
double maxAErr;
void init(Point t, double mae, int w) {
    start = odometry.getPos();
    target = t;
    wait = w;
    doneT = BIL;
    maxAErr = mae;
}
}  // namespace driveData
void pidDriveLineInit(Point target, double maxAErr, const int wait) {
    // prevent div by 0 errors
    if (target.x == 0.0) target.x = 0.001;
    if (target.y == 0.0) target.y = 0.001;
    drivePid.doneTime = BIL;
    curvePid.doneTime = BIL;
    driveData::init(target, maxAErr, wait);
}
bool pidDriveLine() {
    using driveData::doneT;
    using driveData::maxAErr;
    using driveData::start;
    using driveData::target;
    using driveData::wait;
    g_target = target;
    Point pos(odometry.getX(), odometry.getY());
    Point targetDir = target - pos;
    if (targetDir * (target - start).unit() < 4) targetDir = target - start;
    // error detection
    Point dirOrientation = polarToRect(1, odometry.getA());
    double aErr = dirOrientation.angleBetween(targetDir);
    // allow for driving backwards
    int driveDir = 1;
    if (aErr > PI / 2) {
        driveDir = -1;
        aErr = PI - aErr;
    }
    if (dirOrientation < targetDir) aErr *= -driveDir;
    if (dirOrientation > targetDir) aErr *= driveDir;

    // error correction
    double curA = odometry.getA();
    drivePid.target = 0.0;
    drivePid.sensVal = (target - pos) * targetDir.unit();
    curvePid.target = 0;
    curvePid.sensVal = aErr;
    int turnPwr = clamp((int)curvePid.update(), -8000, 8000);
    int drivePMax = 10000;
    if (fabs(aErr) > maxAErr) drivePMax = 2000;
    int drivePwr = clamp((int)drivePid.update(), -drivePMax, drivePMax);
    // prevent turn saturation
    // if (abs(turnPwr) > 0.2 * abs(drivePwr)) turnPwr = (turnPwr < 0 ? -1 : 1) * 0.2 * abs(drivePwr);
    int dlOut = -drivePwr * driveDir - turnPwr;
    int drOut = -drivePwr * driveDir + turnPwr;
    setDL(dlOut);
    setDR(drOut);
    // printf("%d %d ", dlOut, drOut);

    static Point prevPos(0, 0);
    if (fabs(drivePid.sensVal) < 0.5 && (pos - prevPos).mag() < 0.01) {
        if (doneT > millis()) doneT = millis();
    }
    prevPos = pos;
    bool ret = doneT + wait < millis();
    if (ret) printf("* DONE * ");
    return ret;
}
void pidDriveInit(Point target, const int wait) { pidDriveLineInit(target, BIL, wait); }
bool pidDrive() { pidDriveLine(); }