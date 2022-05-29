#include "main.h"
#include <fstream>
#include <iostream>
using namespace std;

double P, tP, turnP;
double tD, tI, D, I, turnD, turnI, preTheta, preP, preTurn, tPID, PID, turnPID = 0;
bool enable = true;
const double kp = 10;
const double kd = 5;
const double ki = 0.007;

const double tkp = 15;
const double tkd = 10;
const double tki = 0.04;

//double turnkp = 350;
//const double turnkd = 30;
//const double turnki = 0.5;

//double turnkp = 275;
//const double turnkd = 5;
//const double turnki = 0.5;

//double turnkp = 350;
//const double turnkd = 7;
//const double turnki = 0.7;
//



void reset(){
    DLF.tare_position();
    DLB.tare_position();
    DRF.tare_position();
    DRB.tare_position();
}

void setDrive(int left, int right){
//    DLF.move_voltage(left);
//    DRF.move_voltage(right);
//    DLB.move_voltage(left);
//    DRB.move_voltage(right);
    DLF.move(left);
    DRF.move(right);
    DLB.move(left);
    DRB.move(right);
}

bool moveHoldFork;
bool moveHoldClaw, moveHold;
short forkPos;
short clawPos;
void holdFork(void*param){
    while(true){
        if (moveHoldFork){
            forkPos = Fork.get_position();
        }
        while(moveHoldFork){
            Fork.move((forkPos - Fork.get_position())*1.75);
//            pros::lcd::set_text(5, std::to_string((forkPos - Fork.get_position()) * 2.5));
            pros::delay(10);
        }
        pros::delay(10);
    }
}

void holdClaw(void*param){
    while(true){
//        pros::lcd::set_text(4, "this is running");
        if (moveHoldClaw){
            clawPos = claw.get_position();
        }
        while(moveHoldClaw){
            claw.move(clawPos - claw.get_position()*5);
            pros::delay(10);
        }
        pros::delay(10);
    }
}

void hold(void*param){
    while(true){
//        pros::lcd::set_text(4, "this is running");
        if (moveHoldClaw){
            clawPos = claw.get_position();
            forkPos = Fork.get_position();
        }
        while(moveHoldClaw){
            claw.move(clawPos - claw.get_position());
            Fork.move(forkPos - Fork.get_position());
            pros::delay(10);
        }
        pros::delay(15);
    }
}

void data(void*param){
    while(true){
        fstream hello;
        hello.open("/usd/data.txt", ios::app);
        hello << globalPos[0] << " " << globalPos[1] << " " << globalPos[2] << endl;
        hello.close();
        pros::delay(15);
    }
}

double Pwr = 0;
double Trn = 0;
int leftPower = 0;
int rightPower = 0;
void drivemotors(){
    int x = abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    int y = abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));

    double LjoyY = y/10.0;
    double LjoyX = x/10.0;
    if ((y > 5) || (x > 5)) {
        if (y > 85) {
            Pwr = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 3.0;
        } else if (y > 55 && y <= 85) {
            Pwr = (12.7 / (1.0 + exp(-(3.0 / 4.0) * (LjoyY - 6.0)))) * 10.0 - 10.0;
        } else {
            Pwr = 5*pow((1.0/5.5)*(LjoyY), 3.0) * 12.7;
        }
        if (x > 85) {
            Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 3.0;
        }
        else if (x > 55 && x <= 85){
            Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 10.0;
        } else {
            Trn = 5*pow((1.0/5.5)*(LjoyX), 3.0) * 12.7;
        }
    }
    else {
        Pwr = 0;
        Trn = 0;
    }
    if (con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0){
        Pwr = -Pwr;
    }
    if (con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0){
        Trn = -Trn;
    }
    leftPower = Pwr + Trn * 0.8;
    rightPower = Pwr - Trn * 0.8;
    setDrive(leftPower, rightPower);
}

double armPwr = 0;
void liftArm(){
    double armPos = abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    if (armPos > 5.0){
        if (armPos > 85) {
            armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 3.0) * 1;
        }
        else if (armPos > 55 && armPos <= 85){
            armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 10.0) * 1;
        }
        else {
            armPwr = (5*pow((1.0/5.5)*(armPos/10.0), 3.0) * 12.7) * 1;
        }
    }else {
        armPwr = 0;
    }
    if (con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0){
        armPwr = -armPwr;
    }
    lift.move(armPwr);
}

bool in = true;
void moveSub(){
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        claw.move_voltage(-12000);
        moveHoldClaw = false;
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        claw.move_voltage(7000);
        moveHoldClaw = false;
    } else {
        claw.move_voltage(0);
        moveHoldClaw = true;
    }
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
//        Fork.move_absolute(3000, 100);
//        tipcontroller->setTarget(3100);
        Fork.move_voltage(11000);
        moveHoldFork = false;
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
//        Fork.move_absolute(1000, 100);
//        tipcontroller->setTarget(2000);
        Fork.move_voltage(-11000);
        moveHoldFork = false;
    } else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && Fork.get_position() > 10){
        Fork.move_voltage(-12000);
        moveHoldFork = false;
    } else{
        Fork.move_voltage(0);
        moveHoldFork = true;
    }
    if(con.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        in = !in;
    }
    if (lift.get_position() > 400 && in){
        intake.move_voltage(-9600);
    } else{
        intake.move_voltage(0);
    }
}

//void moveJaw(){
//    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
//        claw.move_voltage(-4500);
//        moveHoldClaw = false;
//    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
//        claw.move_voltage(4500);
//        moveHoldClaw = false;
//    } else{
//        claw.move_voltage(0);
//        moveHoldClaw = true;
//    }
//}
//
//void moveTip(){
//    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && Fork.get_position() < 3000){
//        Fork.move_voltage(7500);
//        moveHoldFork = false;
//    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && Fork.get_position() > 100){
//        Fork.move_voltage(-7500);
//        moveHoldFork = false;
//    } else{
//        Fork.move_voltage(0);
//        moveHoldFork = true;
//    }
//}

//float left, right;
////true is right, false is left
//float calcMotorPower(float radius, bool dir, float maxPower){
//    //power[0] = left
//    //power[1] = right
//
//
//    float d = radius * 2;
//    float ratio = (d + 14.5)/(d - 14.5);
//    if (dir){
//        left = maxPower;
//        right = maxPower/ratio;
//    } else{
//        right = maxPower;
//        left = maxPower/ratio;
//    }
//
//    return [left, right];
//}

//float calc(float length, float height){
//    float r = ((pow(length, 2))/(8 * height)) + (height/2);
//    return r;
//}

//double *prePos = position();
//void pathFollow(double * path){
//    double *pos = position();
//    double vector[2] = {pos[0] - prePos[0], pos[1] - prePos[1]};
//    double future[2];
//    vector[0] *= 2;
//    vector[1] *= 2;
//
//    future[0] = pos[0] + vector[0];
//    future[1] = pos[1] + vector[1];
//
//
//
//    prePos = pos;
//}

//0 is x, 1 is y
void PIDMove(double units, double angle, int xY){
    enable = true;
    while(enable) {

        P = units - globalPos[xY];
//        if (units < 0){
//            P = P*-1;
//            globalPos[xY] = globalPos[xY]*-1;
//        }
        I += P;
        D = P - preP;

        tP = globalPos[2] - angle;
        tI += tP;
        tD = tP - preTheta;

        PID = (P * kp) + (D * kd) + (I * ki);
        tPID = (tP * tkp) + (tD * tkd) + (tI * tki);
        pros::lcd::set_text(0, std::to_string(globalPos[0]));
//        pros::lcd::set_text(1, std::to_string(globalPos[1]));
//        pros::lcd::set_text(0, std::to_string(P));
//        pros::lcd::set_text(1, std::to_string(I));
//        pros::lcd::set_text(2, std::to_string(D));
//        pros::lcd::set_text(3, std::to_string(PID));
//        pros::lcd::set_text(4, std::to_string(tPID));
        pros::lcd::set_text(5, std::to_string(globalPos[xY]-units));
//        pros::lcd::set_text(6, std::to_string(globalPos[2]-angle));


        setDrive(PID + tPID, PID - tPID);

        preP = P;
        preTheta = tP;
        if ((units == globalPos[xY]) || (round(abs(globalPos[xY]-units)) < 0.02 && abs(globalPos[2]-angle) < 0.007)) {
            setDrive(0, 0);
            enable = false;
            break;
        }

        pros::delay(1);
    }

//    double ave = pos[0];
//    double ave = (pos[0] + pos[1])/2;
//    double final = ave + distance;

//    double final = pos[0] + units;
//    pros::lcd::set_text(5, std::to_string(final));
//    pros::lcd::set_text(4, std::to_string(ave));
//    while(ave < final){
//        ave = pos[0];
//        if(pos[2] != 0){
//            tp = pos[2];
//            td = tp - preTheta;
//            ti += tp;
//            tPID = (tp * tkp) + (td * tkd) + (ti * tki);
//            pros::lcd::set_text(3, std::to_string(tPID));
//        }
//        p = ave;
//        d = p - preP;
//        i += p;
//        PID = (p * kp) + (d * kd) + (i * ki);
//        pros::lcd::set_text(4, std::to_string(PID));
//        setDrive(PID + tPID, PID - tPID);
//        preP = p;
//        preTheta = pos[2];
//        pros::delay(20);
//        pos = position();
//    }
}

//true is right, false is left
const float cir = p * 2.75;
float leftMotor, rightMotor;
void moveArc(double radians, float length, float height, bool dir, bool reverse, int maxPower, int minPower){
    double *start = straight();

    float radius = ((length)*(length)/(8 * height)) + (height/2);
    float ave = (((start[0] + start[1])/2)/360)*cir;
    float arc = radians * radius;

    float d = radius * 2;
    float ratio = (d + 14.5)/(d - 14.5);
    if (minPower == 0){
        if (dir){
            leftMotor = maxPower;
            rightMotor = maxPower/ratio;
        } else{
            rightMotor = maxPower;
            leftMotor = maxPower/ratio;
        }
    } else if(maxPower == 0 && minPower > 0){
        if (dir){
            leftMotor = ratio * minPower;
            rightMotor = minPower;
        } else{
            rightMotor = ratio * minPower;
            leftMotor = minPower;
        }
    }
    while (true){
        double *pos = straight();
        if (reverse){
            pos[0] = pos[0] * -1;
            pos[1] = pos[1] * -1;
        }
        double posAve = (((pos[0] + pos[1])/2)/360)*cir;
        pros::lcd::set_text(0, std::to_string(pos[0]));
        pros::lcd::set_text(1, std::to_string(pos[1]));
        pros::lcd::set_text(2, std::to_string(posAve));

//        if (abs(posAve - ave) > arc){
//            setDrive(0, 0);
//            break;
//        }
        if (reverse){
            if (abs(posAve + ave) > arc){
                setDrive(0, 0);
                break;
            }else {
                setDrive(leftMotor*-1, rightMotor*-1);
            }

        } else{
            if (abs(posAve - ave) > arc){
                setDrive(0, 0);
                break;
            }else{
                setDrive(leftMotor, rightMotor);
            }

        }
        pros::delay(15);
    }
}

double turnki = 0.4;
double preAngle;
int preVal;
double turnkp = 146;  //185 220
const double turnkd = 0;  //25  15 12.95
const double turnkin = 0.0; //0.6    0.3  1.567
const double turnkif = 0.0; //0.57
const double turnkib = 0.0; //0.49

//neg radians turns right
void PIDTurnAbs(double radians, bool multi, int port, int pos, bool great, int volt){
    enable = true;
    turnI, turnP, turnD = 0;
    while(enable){

        if (multi){
            if(great){
                switch (port) {
                    case 0:
                        if(claw.get_position() > pos){
                            claw.move_voltage(volt);
                        }else{
                            claw.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 1:
                        if(Fork.get_position() > pos){
                            Fork.move_voltage(volt);
                        }else{
                            Fork.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 2:
                        if(lift.get_position() > pos){
                            lift.move_voltage(volt);
                        }else{
                            lift.move_voltage(0);
                            multi = false;
                        }
                        break;
                    default:
                        claw.move_voltage(0);
                        Fork.move_voltage(0);
                        lift.move_voltage(0);
                }
            }else{
                switch (port) {
                    case 0:
                        if(claw.get_position() < pos){
                            claw.move_voltage(volt);
                        }else{
                            claw.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 1:
                        if(Fork.get_position() < pos){
                            Fork.move_voltage(volt);
                        }else{
                            Fork.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 2:
                        if(lift.get_position() < pos){
                            lift.move_voltage(volt);
                        }else{
                            lift.move_voltage(0);
                            multi = false;
                        }
                        break;
                    default:
                        claw.move_voltage(0);
                        Fork.move_voltage(0);
                        lift.move_voltage(0);
                }
            }
        }

        turnP = radians - globalPos[2];
        turnI += turnP;
        turnD = turnP - preTurn;
//
//        ofstream pos;
//        pos.open("/usd/Position.txt", std::ios_base::app);
//        pos << globalPos[2] << " " << DLF.get_voltage() << " " << turnP << " " << preAngle-globalPos[2] << " " << std::endl;
//        pos.close();


        if (abs(globalPos[2]) > abs(radians)){
            turnI = 0;
        }
        if (0.1 < abs(turnP) && abs(turnP) < 0.1){
            turnki = 12;
        }else if(abs(turnP) < 0.15){
            turnkp = 157;
            turnki = 110;
        }

        turnPID = (turnP*turnkp) + (turnD*turnkd) + (turnI*turnki);

//        if(abs(turnP) < 0.05 && abs(turnPID) < 15){
//            turnPID= 15;
//        }
        pros::lcd::set_text(0, std::to_string(DLF.get_voltage()));
        pros::lcd::set_text(1, std::to_string(turnPID));
        pros::lcd::set_text(2, std::to_string(radians - globalPos[2]));
//        pros::lcd::set_text(2, std::to_string(globalPos[0]));
//        pros::lcd::set_text(3, std::to_string(globalPos[1]));
        pros::lcd::set_text(4, std::to_string(globalPos[2]));

//        pros::lcd::set_text(2, std::to_string(preAngle-globalPos[2]));
//        pros::lcd::set_text(3, std::to_string(globalPos[2]));

        if (DLF.get_voltage() < 600 && abs(radians-globalPos[2]) < 0.004){
            enable = false;
            setDrive(0, 0);
            claw.move_voltage(0);
            Fork.move_voltage(0);
            lift.move_voltage(0);
            break;
        }
        if (DLF.get_voltage() < 200 && ((preAngle-globalPos[2]) == 0) && abs(radians-globalPos[2]) < 0.05 && preVal == encoder_left.get_value()){
            enable = false;
            setDrive(0, 0);
            claw.move_voltage(0);
            Fork.move_voltage(0);
            lift.move_voltage(0);
            break;
        }

//        if (abs(turnPID) < 10 && abs(radians-globalPos[2]) < 0.3){
//            if(turnPID > 0){
//                turnPID = 10;
//            } else{
//                turnPID = -10;
//            }
//        }

        setDrive(-turnPID, turnPID);

//        if ((abs(radians - globalPos[2]) > 0.004) &&(abs(DLF.get_voltage()) <= 500)){
//            turnPID = turnPID * 4.75;
//        }

        preTurn = turnP;
        preVal = encoder_left.get_value();
        preAngle = globalPos[2];
        pros::delay(15);
    }
}

double angleE, angleI, angleD, driveE, driveI, driveD, preAE, preDE, anglePID, drivePID = 0;
double angleKp = 198;
double angleKi = 0.24;
//double angleKp = 0;
//double angleKi = 0;
double angleKd = 0;
double driveKp = 4.6;
double driveKi = 0.008;
double driveKd = 0;
double scalar;
double Min = 30;
double leftMove, rightMove = 0;
//for port 0 is claw, 1 is fork, 2 is lift
//great means if current position is greater than position provided
void MoveToPoint(double x, double y, double heading, double Max, bool dir, bool ring, bool multi, int port, int pos, bool great, int volt){
    enable = true;
    while(enable){
        double tM = atan2(x-globalPos[0], y-globalPos[1]);
        double startX = globalPos[0];
        double startY = globalPos[1];
        double startH = globalPos[2];

        ofstream position;
        position.open("/usd/Position.txt", std::ios_base::app);
        position << globalPos[0] << " " << globalPos[1] << " " << globalPos[2] << " " << drivePID << " " << anglePID << " " << driveE << " " << angleE << " " << leftMove << " " << rightMove << " " << tM << std::endl;
        position.close();

        driveE = sqrt(std::pow((y-startY), 2)+std::pow(x-startX, 2));
        driveI += driveE;
        driveD = driveE - preDE;
        drivePID = (driveE * driveKp) + (driveI * driveKi) + (driveD * driveKd);

        angleE = tM - startH;
        angleI += angleE;
        angleD = angleE - preAE;
        anglePID = (angleE * angleKp) + (angleI * angleKi) + (angleD * angleKd);

        if (multi){
            if(great){
                switch (port) {
                    case 0:
                        if(claw.get_position() > pos){
                            claw.move_voltage(volt);
                        }else{
                            claw.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 1:
                        if(Fork.get_position() > pos){
                            Fork.move_voltage(volt);
                        }else{
                            Fork.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 2:
                        if(lift.get_position() > pos){
                            lift.move_voltage(volt);
                        }else{
                            lift.move_voltage(0);
                            multi = false;
                        }
                        break;
                    default:
                        claw.move_voltage(0);
                        Fork.move_voltage(0);
                        lift.move_voltage(0);
                }
            }else{
                switch (port) {
                    case 0:
                        if(claw.get_position() < pos){
                            claw.move_voltage(volt);
                        }else{
                            claw.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 1:
                        if(Fork.get_position() < pos){
                            Fork.move_voltage(volt);
                        }else{
                            Fork.move_voltage(0);
                            multi = false;
                        }
                        break;
                    case 2:
                        if(lift.get_position() < pos){
                            lift.move_voltage(volt);
                        }else{
                            lift.move_voltage(0);
                            multi = false;
                        }
                        break;
                    default:
                        claw.move_voltage(0);
                        Fork.move_voltage(0);
                        lift.move_voltage(0);
                }
            }
        }
        if(lift.get_position() > 400 && ring){
            intake.move_voltage(-12000);
        }

//        pros::lcd::set_text(0, std::to_string(anglePID));
//        pros::lcd::set_text(1, std::to_string(drivePID));
//        pros::lcd::set_text(2, std::to_string(angleE));
//        pros::lcd::set_text(3, std::to_string(driveE));

//        if(abs(angleE) < 0.15){
//            anglePID = 0;
//        }
        leftMove = drivePID - anglePID;
        rightMove = drivePID + anglePID;

        if (abs(leftMove) > Max || abs(rightMove) > Max){
            if (abs(leftMove) > abs(rightMove)){
                scalar = Max/leftMove;
            }else{
                scalar = Max/rightMove;
            }
            leftMove *= scalar;
            rightMove *= scalar;
        }
//        else if (abs(leftMove) < Min && abs(rightMove) < Min){
//            leftMove = 30;
//            rightMove = 30;
//        }
//        pros::lcd::set_text(4, std::to_string(leftMove));
//        pros::lcd::set_text(5, std::to_string(rightMove));

        if (dir){
            if (((abs(x-globalPos[0])) < 0.3) && ((abs(y-globalPos[1])) < 0.3)){
                PIDTurnAbs(heading);
                dir = false;
                claw.move_voltage(0);
                Fork.move_voltage(0);
                lift.move_voltage(0);
                break;
            }
        }else{
            if (((abs(x-globalPos[0])) < 3) && ((abs(y-globalPos[1])) < 3)){
                setDrive(0, 0);
                enable = false;
                claw.move_voltage(0);
                Fork.move_voltage(0);
                lift.move_voltage(0);
                break;
            }
        }
        setDrive(leftMove, rightMove);

        preAE = angleE;
        preDE = driveE;
        pros::delay(20);
    }
}

float leftPow, rightPow = 0;
void arcFollow(std::vector<double>& x, std::vector<double>& y, double heading, int max, bool dir){
//    double startX = globalPos[0];
//    double startY = globalPos[1];
//    double startH = globalPos[2];
//
//    double
    for (int j=0; j<=x.size()-1; j++){
        double one_two = sqrt(std::pow((x[j+1]-x[j]), 2)+std::pow((y[j+1]-y[j]), 2));
        double bottom = x[j+1]-x[j];
        double angle = 2*acos(bottom/one_two);
        double radius = bottom/sin(angle);
        double ratio = ((radius*2)+7.25)/((radius*2)-7.25);
//        pros::lcd::set_text(0, std::to_string(one_two));
//        pros::lcd::set_text(1, std::to_string(bottom));
//        pros::lcd::set_text(2, std::to_string(angle));
//        pros::lcd::set_text(3, std::to_string(radius));
        pros::lcd::set_text(2, std::to_string(ratio));

        if(x[j+1]>x[j]){
            rightPow = max;
            leftPow = max/ratio;
        } else{
            rightPow = max/ratio;
            leftPow = max;
        }
        while(true){
            pros::lcd::set_text(0, std::to_string(leftPow));
            pros::lcd::set_text(1, std::to_string(rightPow));
            setDrive(leftPow, rightPow);
            if (((abs(x[j+1]-globalPos[0])) < 1) && ((abs(y[j+1]-globalPos[1])) < 1)){
                setDrive(0, 0);
                break;
            }
            pros::delay(20);
        }
    }
}

void arcFind(std::vector<double> &x, std::vector<double> &y){

}

