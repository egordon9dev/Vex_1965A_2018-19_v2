#include "pid.hpp"
#include "main.h"
#include "setup.hpp"
using std::cout;
Pid_t flywheelPid, clawPid, drfbPid, DLPid, DRPid, drivePid, turnPid, curvePid, intakePid, curveVelPid, sTurnPid;
Slew_t flywheelSlew, drfbSlew, DLSlew, DRSlew, clawSlew, intakeSlew;
/*
 ########  #### ########           ######  ##       ######## ##      ##
 ##     ##  ##  ##     ##         ##    ## ##       ##       ##  ##  ##
 ##     ##  ##  ##     ##         ##       ##       ##       ##  ##  ##
 ########   ##  ##     ## ####     ######  ##       ######   ##  ##  ##
 ##         ##  ##     ## ####          ## ##       ##       ##  ##  ##
 ##         ##  ##     ##  ##     ##    ## ##       ##       ##  ##  ##
 ##        #### ########  ##       ######  ######## ########  ###  ###
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
    derivativeUpdateInterval = prevTime = prevDUpdateTime = 0;
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
    if (fabs(turnPid.sensVal - turnPid.target) < 0.1 && fabs(getDLVel()) < 0.1 && fabs(getDRVel()) < 0.1) {
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
/*
  ######  ##      ## ######## ######## ########
 ##    ## ##  ##  ## ##       ##       ##     ##
 ##       ##  ##  ## ##       ##       ##     ##
  ######  ##  ##  ## ######   ######   ########
       ## ##  ##  ## ##       ##       ##
 ##    ## ##  ##  ## ##       ##       ##
  ######   ###  ###  ######## ######## ##
*/
namespace sweep {
Point start, target;
int wait, doneT;
double biasL, biasR;
double maxAErr;
void init(double tL, double tR, double bias, int wait) {
    double r = fabs(tL / tR);
    if (bias == 0) bias = 0.000001;
    double tgt = 0.0;
    if (r > 1.0) {
        tgt = tR;
        biasL = bias;
        biasR = 1 / bias;
        bentOdo.setScales(1 / r, 1.0);
    } else {
        tgt = tL;
        biasL = 1 / bias;
        biasR = bias;
        bentOdo.setScales(1.0, r);
    }
    bentOdo.reset();
    bentOdo.setX(0);
    bentOdo.setY(0);
    bentOdo.setA(PI / 2);
    sweep::wait = wait;
    sweep::doneT = BIL;
    sweep::start = Point(0, 0);
    sweep::target = Point(0, tgt);
    sweep::maxAErr = 0.2;
}
}  // namespace sweep
void pidSweepInit(double tL, double tR, int bias, int wait) { sweep::init(tL, tR, bias, wait); }
bool pidSweep() {
    Point pos = bentOdo.getPos();
    using sweep::doneT;
    using sweep::maxAErr;
    using sweep::start;
    using sweep::target;
    using sweep::wait;

    //---- Do math as if we were driving straight because we bent space wooooOOOoohooo

    Point targetDir = target - pos;
    if (targetDir * (target - start).unit() < 4) targetDir = target - start;
    // error detection
    Point dirOrientation = polarToRect(1, bentOdo.getA());
    double aErr = dirOrientation.angleBetween(targetDir);
    // allow for driving backwards
    int driveDir = target.y < 0 ? -1 : 1;
    if (driveDir == -1) { aErr = PI - aErr; }
    if (dirOrientation < targetDir) aErr *= -driveDir;
    if (dirOrientation > targetDir) aErr *= driveDir;
    // error correction
    double curA = bentOdo.getA();
    drivePid.target = 0.0;
    drivePid.sensVal = (target - pos) * targetDir.unit();
    int drivePMax = 10000;
    double drivePwr = drivePid.update();
    if (fabs(aErr) > maxAErr) drivePwr = 0.0;
    if (fabs(getDriveVel()) < 0.2 && drivePwr * driveDir > 0 && fabs(drivePid.sensVal) > (target - start).mag() * 0.5) drivePwr = clamp(drivePwr, -2500.0, 2500.0);
    // by updating both pids, we keep the derivative and integral terms updated so they don't get intermitent data
    bool useTurnPid = fabs(getDriveVel()) < 0.15 || fabs(aErr) > maxAErr;
    turnPid.target = 0;
    turnPid.sensVal = aErr;
    double turnPidOutput = turnPid.update();
    curvePid.target = 0;
    curvePid.sensVal = aErr;
    double curvePidOutput = curvePid.update();
    if (!useTurnPid) turnPid.errTot = 0;
    drivePwr = clamp((int)drivePwr, -drivePMax, drivePMax);
    double curveInfluence = 1.5;
    int maxCurvePwr = lround(curveInfluence * fabs(drivePwr));
    int rotPwr = useTurnPid ? clamp(lround(turnPidOutput), -12000, 12000) : clamp(lround(curvePidOutput), -maxCurvePwr, maxCurvePwr);
    // if (fabs(drivePid.sensVal) < 6) rotPwr = 0;
    // printf("{%+5d %+5d %s}", (int)lround(drivePwr), (int)lround(rotPwr), useTurnPid ? "turnPid" : "curvePid");
    int dlOut = -drivePwr * driveDir - rotPwr;
    int drOut = -drivePwr * driveDir + rotPwr;
    setDL(sweep::biasL * dlOut);
    setDR(sweep::biasR * drOut);

    static Point prevPos(0, 0);
    if (fabs(drivePid.sensVal) < 0.5 && fabs(getDriveVel()) < 0.2) {
        if (doneT > millis()) doneT = millis();
    }
    prevPos = pos;
    bool ret = doneT + wait < millis();
    if (ret) printf("* DONE * ");
    return ret;
}
/*
 ########  ########  #### ##     ## ########
 ##     ## ##     ##  ##  ##     ## ##
 ##     ## ##     ##  ##  ##     ## ##
 ##     ## ########   ##  ##     ## ######
 ##     ## ##   ##    ##   ##   ##  ##
 ##     ## ##    ##   ##    ## ##   ##
 ########  ##     ## ####    ###    ########
*/
Point g_target;
Point driveData::start, driveData::target;
int driveData::wait, driveData::doneT;
double driveData::maxAErr;
bool driveData::flip;
void driveData::init(Point s, Point t, bool f, double mae, int w) {
    start = s;
    target = t;
    wait = w;
    doneT = BIL;
    maxAErr = mae;
    flip = f;
}
void setMaxAErr(double mae) { driveData::maxAErr = mae; }
void pidDriveLineInit(Point start, Point target, bool flip, double maxAErr, const int wait) {
    // prevent div by 0 errors
    if (target.x == 0.0) target.x = 0.001;
    if (target.y == 0.0) target.y = 0.001;
    drivePid.doneTime = BIL;
    curvePid.doneTime = BIL;
    driveData::init(start, target, flip, maxAErr, wait);
    // if (flip) {
    //     drivePid.kp = 1000;
    //     drivePid.ki = 7;
    //     drivePid.kd = 80000;
    // } else {
    //     drivePid.kp = 1100;
    //     drivePid.ki = 3;
    //     drivePid.kd = 80000;
    // }
}
bool pidDriveLine() {
    using driveData::doneT;
    using driveData::maxAErr;
    using driveData::start;
    using driveData::target;
    using driveData::wait;
    g_target = target;
    Point pos = odometry.getPos();
    Point targetDir = target - pos;
    if (targetDir * (target - start).unit() < 4) targetDir = target - start;
    // error detection
    Point dirOrientation = polarToRect(1, odometry.getA());
    double aErr = dirOrientation.angleBetween(targetDir);
    // allow for driving backwards
    int driveDir = driveData::flip ? -1 : 1;
    if (driveDir == -1) { aErr = PI - aErr; }
    if (dirOrientation < targetDir) aErr *= -driveDir;
    if (dirOrientation > targetDir) aErr *= driveDir;
    // error correction
    double curA = odometry.getA();
    drivePid.target = 0.0;
    drivePid.sensVal = (target - pos) * targetDir.unit();
    int drivePMax = driveDir == -1 ? 12000 : 12000;
    double drivePwr = drivePid.update();
    if (fabs(aErr) > maxAErr) drivePwr = 0.0;
    if (fabs(getDriveVel()) < 0.2 && drivePwr * driveDir > 0 && fabs(drivePid.sensVal) > (target - start).mag() * 0.5) drivePwr = clamp(drivePwr, -2500.0, 2500.0);
    // by updating both pids, we keep the derivative and integral terms updated so they don't get intermitent data
    bool useTurnPid = fabs(getDriveVel()) < 0.15 || fabs(aErr) > maxAErr;
    turnPid.target = 0;
    turnPid.sensVal = aErr;
    double turnPidOutput = turnPid.update();
    curvePid.target = 0;
    curvePid.sensVal = aErr;
    double curvePidOutput = curvePid.update();
    if (!useTurnPid) turnPid.errTot = 0;
    drivePwr = clamp((int)drivePwr, -drivePMax, drivePMax);
    double curveInfluence = 1.5;
    int maxCurvePwr = lround(curveInfluence * fabs(drivePwr));
    int rotPwr = useTurnPid ? clamp(lround(turnPidOutput), -12000, 12000) : clamp(lround(curvePidOutput), -maxCurvePwr, maxCurvePwr);
    // if (fabs(drivePid.sensVal) < 6) rotPwr = 0;
    // printf("{%+5d %+5d %s}", (int)lround(drivePwr), (int)lround(rotPwr), useTurnPid ? "turnPid" : "curvePid");
    int dlOut = -drivePwr * driveDir - rotPwr;
    int drOut = -drivePwr * driveDir + rotPwr;
    setDL(dlOut);
    setDR(drOut);

    static Point prevPos(0, 0);
    if (fabs(drivePid.sensVal) < 0.5 && fabs(getDriveVel()) < 0.2) {
        if (doneT > millis()) doneT = millis();
    }
    prevPos = pos;
    bool ret = doneT + wait < millis();
    if (ret) printf("* DONE * ");
    return ret;
}
void pidDriveInit(Point target, const int wait) {
    Point pos = odometry.getPos();
    double aErr = (target - pos).angleBetween(polarToRect(1, odometry.getA()));
    pidDriveLineInit(odometry.getPos(), target, aErr > PI / 2 ? true : false, BIL, wait);
}
bool pidDrive() { pidDriveLine(); }