#include "main.h"

double P, tP, turnP;
double tD, tI, D, I, turnD, turnI, preTheta, preP, preTurn, tPID, PID, turnPID = 0;
bool enable = true;
const double kp = 15;
const double kd = 10;
const double ki = 0;

const double tkp = 17;
const double tkd = 5;
const double tki = 0;

const double turnkp = 130;
const double turnkd = 11;
const double turnki = 0;

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
//    DLF = left;
//    DRF = right;
//    DLB = left;
//    DRB = right;
}

bool moveHoldFork, moveHoldClaw, moveHold;
short forkPos;
short clawPos;
void holdFork(void*param){
    while(true){
        pros::lcd::set_text(5, "this is running");
        if (moveHoldFork){
            forkPos = Fork.get_position();
        }
        while(moveHoldFork){
            Fork.move((forkPos - Fork.get_position())*2);
            pros::delay(10);
        }
        pros::delay(10);
    }
}

void holdClaw(void*param){
    while(true){
        pros::lcd::set_text(4, "this is running");
        if (moveHoldClaw){
            clawPos = claw.get_position();
        }
        while(moveHoldClaw){
            claw.move(clawPos - claw.get_position());
            pros::delay(10);
        }
        pros::delay(10);
    }
}

void hold(void*param){
    while(true){
        pros::lcd::set_text(4, "this is running");
        if (moveHoldClaw){
            clawPos = claw.get_position();
            forkPos = Fork.get_position();
        }
        while(moveHoldClaw){
            claw.move(clawPos - claw.get_position());
            Fork.move(forkPos - Fork.get_position());
            pros::delay(10);
        }
        pros::delay(10);
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
    leftPower = Pwr + Trn;
    rightPower = Pwr - Trn;
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

void moveSub(){
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && claw.get_position() < -50){
        claw.move_voltage(4500);
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && claw.get_position() > -300){
        claw.move_voltage(-4500);
    } else {
        claw.move_voltage(0);
    }
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && Fork.get_position() < 3000){
        pros::lcd::set_text(0, "L2 pressed");
        Fork.move_voltage(9000);
        moveHoldFork = false;
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && Fork.get_position() > 100){
        Fork.move_voltage(-9000);
        moveHoldFork = false;
    } else{
        Fork.move_voltage(0);
        moveHoldFork = true;
    }
}

void moveJaw(){
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        claw.move_voltage(-4500);
        moveHoldClaw = false;
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        claw.move_voltage(4500);
        moveHoldClaw = false;
    } else{
        claw.move_voltage(0);
        moveHoldClaw = true;
    }
}

void moveTip(){
    if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && Fork.get_position() < 3000){
        Fork.move_voltage(7500);
        moveHoldFork = false;
    } else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && Fork.get_position() > 100){
        Fork.move_voltage(-7500);
        moveHoldFork = false;
    } else{
        Fork.move_voltage(0);
        moveHoldFork = true;
    }
}

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

void PIDMove(double units){
    while(enable) {
        double *pos = position();
        P = units - pos[0];
        I += P;
        D = P - preP;

        tP = pos[2];
        tI += tP;
        tD = tP - preTheta;

        PID = (P * kp) + (D * kd) + (I * ki);
        tPID = (tP * tkp) + (tD * tkd) + (tI * tki);
        pros::lcd::set_text(0, std::to_string(pos[0]));
        pros::lcd::set_text(1, std::to_string(pos[2]));
//        pros::lcd::set_text(0, std::to_string(P));
//        pros::lcd::set_text(1, std::to_string(I));
//        pros::lcd::set_text(2, std::to_string(D));
        pros::lcd::set_text(3, std::to_string(PID));
        pros::lcd::set_text(4, std::to_string(tPID));

//        if (units < 0){
//            PID = PID * -1;
//            tPID = tPID * -1;
//        }

        setDrive(PID + tPID, PID - tPID);
        preP = P;
        preTheta = tP;
        if ((units == pos[0]) || (((P < 0.03) && (P > -0.03)))) {
            enable = false;
            break;
        }
        pros::delay(20);
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


void PIDTurn(double radians){
    while(enable){
        double *pos = position();

        turnP = radians - pos[2];
        turnI += turnP;
        turnD = turnP - preTurn;
        turnPID = (turnP * turnkp) + (turnD * turnkd) + (turnI * turnki);
        setDrive(-1*turnPID, 1*turnPID);
//        pros::lcd::set_text(0, std::to_string(P));
//        pros::lcd::set_text(1, std::to_string(I));
//        pros::lcd::set_text(2, std::to_string(D));
        pros::lcd::set_text(2, std::to_string(turnPID));
        pros::lcd::set_text(3, std::to_string(pos[2]));
        if (radians == pos[2] || ((turnP < 0.05) && (turnP > -0.05))) {
            enable = false;
            break;
        }
        preTurn = turnP;
        pros::delay(20);
    }
}

// working only with global positions
// 2d array of row 1 being starting coords
// row 2 being ending coords
// col 1 x, col 2 y, col 3 angle
// Theta = tan inverse ((m1 - m2)/(1 + m1m2))
// setDrive(127, 2.15); drive in a circle 15 inches in diameter or 7.5 inches in radius
// startingL[2] = x, y

//float startingL[2], startingR[2], finishL, finishR;
//void dubinsCurve(float coords[][3], bool reverse){
//    //calculate the 4 circles
//    if ((coords[0][2] >= 0) && (coords[0][2] <= 90)){
//        if (coords[0][2] == 90){
//            startingL[0] = coords[0][0];
//
//        }
//    }
//    if ((coords[0][2] == 0) || (coords[0][2] == 180)){
//        startingL[0] = coords[0][0] + 7.5;
//        startingL[1] = coords[0][1], startingR[1] = coords[0][1];
//        startingR[0] = coords[0][0] - 7.5;
//    } else if ((coords[0][2] == 90) || (coords[0][2] == 270)){
//        startingL[0] = coords[0][0], startingR[0] = coords[0][0];
//        startingL[1] = coords[0][1] - 7.5;
//        startingR[1] = coords[0][1] + 7.5;
//    } else if (()){
//        float m2 = tan()
//    }
////    float startingL =
//}