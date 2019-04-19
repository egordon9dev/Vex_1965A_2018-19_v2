#ifndef MOTOR_SAVER_H
#define MOTOR_SAVER_H
#include <deque>
class MotorSaver {
   private:
    int speedsLen;
    int avgPwr;
    int pwr1, pwr2;
    double avgSpeed;
    double spd1, spd2;
    std::deque<int> pwrs;
    std::deque<double> speeds;
    MotorSaver();

   public:
    MotorSaver(int iterations);
    ~MotorSaver();
    void setConstants(int p1, int p2, double s1, double s2);
    int getPwr(int inputPwr, double vel);
    bool isFaster(double d);
    bool isPwr(int d);
    void reset();
};

#endif
