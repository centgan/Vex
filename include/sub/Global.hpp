#include "main.h"

//globalizing motors
extern pros::Motor DLF;
extern pros::Motor DRF;
extern pros::Motor DLB;
extern pros::Motor DRB;
extern pros::Motor lift;
extern pros::Motor claw;
extern pros::Motor Fork;

//globalizing okapi motors
//extern MotorGroup left;
//extern MotorGroup right;
//extern Motor arm;
//extern Motor jaw;
//extern Motor tip;

//globalizing encoders
extern pros::ADIEncoder encoder_right;
extern pros::ADIEncoder encoder_left;
extern pros::ADIEncoder encoder_rear;

//globalizing sensors
extern pros::Imu imu_sensor;

//globalizing controller
extern pros::Controller con;

//globalizing okapi controller
extern Controller controller;
