#ifndef MOTOR_SAVER_H
#define MOTOR_SAVER_H

class MotorSaver {
   private:
    int prevEncVal, speedsLen, maxSpeed, maxPwr, sumSpeed, sumPwr;
    double pwr1, pwr2, spd1, spd2;
    int *speeds;
    int *pwrs;
    MotorSaver();

   public:
    MotorSaver(int iterations);
    ~MotorSaver();
    void setConstants(double p1, double p2, double s1, double s2);
    int getPwr(int inputPwr, int encoderValue);
    int getSumSpeed();
    int getSumPwr();
    int getMaxSpeed();
    int getMaxPwr();
    bool isFaster(double d);
    bool isPwr(double d);
    void reset();
};

#endif
