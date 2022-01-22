#include "main.h"

//constants
const double circ = (p * 2.75)/360;
const double L = 9.25;
const double B = 3;

//left, right, back, heading encoder values
double prePos[3] = {0, 0, 0};
double curPos[3] = {0, 0, 0};

//x position, y position, heading
double globalPos[3];

void inertial(){
//   //pros::Imu imu_sensor(18);
//    int time = pros::millis();
//    int iter = 0;
////    int stp1 = 0;
////    while (imu_sensor.is_calibrating()) {
////        pros::lcd::set_text(3, "Calibrating...");
////        stp1 += 10;
////        pros::delay(10);
////    }
//    pros::lcd::set_text(3, std::to_string(iter));
//    pros::lcd::set_text(4, std::to_string(time));
////    iter = 0;
//    while (iter - time < 5000) {
//        pros::lcd::set_text(5, std::to_string(imu_sensor.get_heading()));
//        iter += 20;
//        pros::delay(20);
//    }
//    pros::lcd::set_text(6, std::to_string(iter-time));
////    imu_sensor.reset();
//   // should print about 2000 ms
//    //printf("IMU is done calibrating (took %d ms)\n", iter - time);
////    while (true){
////        pros::lcd::set_text(1, std::to_string(imu_sensor.get_heading()));
////        pros::delay(20);
////    }
////    // should print about 2000 ms
////    pros::lcd::set_text(2, std::to_string(iter - time));
}
//
void Sensors_reset(){
    encoder_left.reset();
    encoder_right.reset();
    encoder_rear.reset();
}

double * straight(){
    curPos[0] = encoder_left.get_value();
    curPos[1] = encoder_right.get_value();
    curPos[2] = encoder_rear.get_value();
    return curPos;
}

void position(void*param){
    while(true){
        static bool once = [](){
            globalPos[0] = 0;
            globalPos[1] = 0;
            globalPos[2] = 0;
            return true;
        } ();
        curPos[0] = encoder_left.get_value();
        curPos[1] = encoder_right.get_value();
        curPos[2] = encoder_rear.get_value();

        double n1 = curPos[0] - prePos[0];
        double n2 = curPos[1] - prePos[1];
        double n3 = curPos[2] - prePos[2];

//    pros::lcd::set_text(0, std::to_string(curPos[0]));
//    pros::lcd::set_text(1, std::to_string(curPos[1]));
//    pros::lcd::set_text(2, std::to_string(curPos[2]));

        double x = circ * ((n1 + n2)/2);
        double y = circ * (n3 - (B * (n2 - n1)/L));
        double theta = circ * (n2 - n1)/L;
//    double degree = (theta * 180)/pi;
//        pros::lcd::set_text(3, std::to_string(x));
//        pros::lcd::set_text(4, std::to_string(y));
//        pros::lcd::set_text(5, std::to_string(theta));
//        pros::lcd::set_text(6, std::to_string(degree));


        globalPos[0] += x * cos(globalPos[2] + theta/2) - y * sin(globalPos[2] + theta/2);
        globalPos[1] += x * sin(globalPos[2] + theta/2) + y * cos(globalPos[2] + theta/2);
        globalPos[2] += theta;


        pros::lcd::set_text(3, std::to_string(globalPos[0]));
//    pros::lcd::set_text(4, std::to_string(globalPos[1]));
//    pros::lcd::set_text(5, std::to_string(globalPos[2]));

        prePos[0] = curPos[0];
        prePos[1] = curPos[1];
        prePos[2] = curPos[2];
        pros::delay(2);
    }
}