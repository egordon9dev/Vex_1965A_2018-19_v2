#include "MotorSaver.hpp"
#include "setup.hpp"
MotorSaver::MotorSaver(){};
MotorSaver::MotorSaver(int iterations) {
    speedsLen = iterations;
    setConstants(12000, 12000, 0.0, 0.0);
}
void MotorSaver::setConstants(int p1, int p2, double s1, double s2) {
    pwr1 = p1;
    pwr2 = p2;
    spd1 = s1;
    spd2 = s2;
}
MotorSaver::~MotorSaver() {}
int MotorSaver::getPwr(int inputPwr, double vel) {
    speeds.push_back(vel);
    while (speeds.size() > speedsLen) speeds.pop_front();
    pwrs.push_back(inputPwr);
    while (pwrs.size() > speedsLen) pwrs.pop_front();
    avgSpeed = avgPwr = 0.0;
    for (const auto& p : pwrs) { avgPwr += p; }
    for (const auto& s : speeds) { avgSpeed += s; }
    avgSpeed /= speedsLen;
    avgPwr /= speedsLen;
    if (speeds.size() == speedsLen) {
        if (pros::competition::is_autonomous() || getDrfb() < drfb18Max + 80) {
            if (avgSpeed < spd1 && avgPwr > pwr1 || avgSpeed > -spd1 && avgPwr < -pwr1) { inputPwr = clamp(inputPwr, -pwr1, pwr1); }
            if (avgSpeed < spd2 && avgPwr > pwr2 || avgSpeed > -spd2 && avgPwr < -pwr2) { inputPwr = clamp(inputPwr, -pwr2, pwr2); }
        }
    }
    return clamp(inputPwr, -12000, 12000);
}
void MotorSaver::reset() {
    for (int i = 0; i < speedsLen; i++) speeds[i] = pwrs[i] = 0;
}
bool MotorSaver::isFaster(double d) { return avgSpeed > d; }
bool MotorSaver::isPwr(int d) { return avgPwr > d; }