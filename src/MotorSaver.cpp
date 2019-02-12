#include "MotorSaver.hpp"
#include "setup.hpp"
MotorSaver::MotorSaver(){};
MotorSaver::MotorSaver(int iterations) {
    speedsLen = iterations;
    speeds = new int[iterations];
    pwrs = new int[iterations];
    maxSpeed = 30 * speedsLen;
    maxPwr = 12000 * speedsLen;
    setConstants(0.7, 0.4, 0.1, 0.01);
}
void MotorSaver::setConstants(double p1, double p2, double s1, double s2) {
    pwr1 = p1;
    pwr2 = p2;
    spd1 = s1;
    spd2 = s2;
}
MotorSaver::~MotorSaver() {
    delete[] speeds;
    delete[] pwrs;
}
int MotorSaver::getPwr(int inputPwr, int encoderValue) {
    static bool prevLim1 = false, prevLim2 = false;
    static int prevDir = 0;
    // record array of speeds
    static bool first = true;
    if (first) {
        for (int i = 0; i < speedsLen; i++) speeds[i] = pwrs[i] = 0;
    }
    first = false;
    for (int i = 0; i < speedsLen - 1; i++) {
        speeds[i] = speeds[i + 1];
        pwrs[i] = pwrs[i + 1];
    }
    speeds[speedsLen - 1] = abs(encoderValue - prevEncVal);
    prevEncVal = encoderValue;
    pwrs[speedsLen - 1] = abs(inputPwr);

    // limit stall torque to protect motor
    sumSpeed = 0, sumPwr = 0;
    for (int i = 0; i < speedsLen; i++) {
        sumSpeed += speeds[i];
        sumPwr += pwrs[i];
    }
    int pLim1 = maxPwr * pwr1, pLim2 = maxPwr * pwr2;
    if (sumSpeed < maxSpeed * spd1 && sumPwr > pLim1) { inputPwr = clamp(inputPwr, -pLim1, pLim1); }
    if (sumSpeed < maxSpeed * spd2 && sumPwr > pLim2) { inputPwr = clamp(inputPwr, -pLim2, pLim2); }
    return clamp(inputPwr, -12000, 12000);
}
void MotorSaver::reset() {
    for (int i = 0; i < speedsLen; i++) speeds[i] = pwrs[i] = 0;
}
int MotorSaver::getSumSpeed() { return sumSpeed; }
int MotorSaver::getSumPwr() { return sumPwr; }
int MotorSaver::getMaxSpeed() { return maxSpeed; }
int MotorSaver::getMaxPwr() { return maxPwr; }
bool MotorSaver::isFaster(double d) { return sumSpeed > maxSpeed * d; }
bool MotorSaver::isPwr(double d) { return sumPwr > maxPwr * d; }