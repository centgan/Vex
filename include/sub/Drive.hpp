#include "main.h"

void reset();

void setDrive(int left, int right);

void hold(void*param);

void holdClaw(void*param);

void holdFork(void*param);

void drivemotors();

void liftArm();

void moveSub();

void moveJaw();

void moveTip();

void PIDMove(double units, double angle, int xY);

void moveArc(double radians, float length, float height, bool dir, bool reverse = false, int maxPower = 0, int minPower = 0);

void PIDTurnAbs(double radians, int weight);

void PIDTurnRel(double radians);

//void dubinsCurve(float coords[][3], bool reverse);

extern bool moveHoldFork;

