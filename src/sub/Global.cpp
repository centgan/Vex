#include "main.h"

pros::Motor DLF(19, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DRF(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DLB(13, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DRB(14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor lift(11, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor claw(18, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Fork(15, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//Initializing okapi Motors
//okapi::MotorGroup left({13, 11});
//okapi::MotorGroup right({-18, 20});
//okapi::Motor arm(-12);
//okapi::Motor jaw(-17);
//okapi::Motor tip(-15);


//Initializing Encoders
//pros::ADIEncoder encoder_right(7, 8, true);
//pros::ADIEncoder encoder_left(5, 6, false);
//pros::ADIEncoder encoder_rear(3, 4, false);

pros::ADIEncoder encoder_right(3, 4, false);
pros::ADIEncoder encoder_left(5, 6, true);
pros::ADIEncoder encoder_rear(7, 8, false);

//Initializing Sensors
pros::Imu imu_sensor(18);

//Initializing Controller
pros::Controller con(pros::E_CONTROLLER_MASTER);

//Initializing okapi Controller
Controller controller;

const double p = 2 * acos(0.0);


