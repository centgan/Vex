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

void PIDMove(double units);

void PIDTurn(double radians);

//void dubinsCurve(float coords[][3], bool reverse);

