#include "main.h"

void reset();

void setDrive(int left, int right);

void hold(void*param);

void holdClaw(void*param);

void holdFork(void*param);

void data(void*param);

void drivemotors();

void liftArm();

void moveSub();

void moveJaw();

void moveTip();

void PIDMove(double units, double angle, int xY);

void moveArc(double radians, float length, float height, bool dir, bool reverse = false, int maxPower = 0, int minPower = 0);

void PIDTurnAbs(double radians, bool multi=false, int port=0, int pos=0, bool great=false, int volt=0);

void MoveToPoint(double x, double y, double heading, double Max, bool dir, bool ring=false, bool multi=false, int port=0, int pos=0, bool great=false, int volt=0);

void diff(double x, double y, double heading, int max, bool dir);
//void dubinsCurve(float coords[][3], bool reverse);

void arcFollow(std::vector<double>& x, std::vector<double>& y, double heading, int max, bool dir);

extern bool moveHoldFork;

