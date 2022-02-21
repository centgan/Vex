#include "main.h"

MotorGroup left({19, 13});
MotorGroup right({9, 14});
std::shared_ptr<OdomChassisController> chassis =
        ChassisControllerBuilder()
                .withMotors(left, right) // left motor is 1, right motor is 2 (reversed)
                .withGains(
                        {0.0008, 0.0004, 0.00005}, // distance controller gains 0.00001
                        {0.0008, 0, 0.00001}, // turn controller gains
                        {0.0008, 0.000001, 0.00005}  // angle controller gains (helps drive straight)0.0001
                )
                .withSensors(
                        ADIEncoder{'E', 'F'}, // left encoder in ADI ports A & B
                        ADIEncoder{'G', 'H'},  // right encoder in ADI ports C & D (reversed)
                        ADIEncoder{'C', 'D'}  // middle encoder in ADI ports E & F
                )
                        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
                        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
                .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 9.5_in, 4_in, 2.75_in}, quadEncoderTPR})
                .withOdometry() // use the same scales as the chassis (above)
                .buildOdometry(); // build an odometry chassis

std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
                .withMotors(left, right)
                .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.5_in}, imev5GreenTPR})
                .build();

std::shared_ptr<AsyncMotionProfileController> profileControllers =
        AsyncMotionProfileControllerBuilder()
                .withLimits({
                                    0.25, // Maximum linear velocity of the Chassis in m/s
                                    0.5, // Maximum linear acceleration of the Chassis in m/s/s
                                    2.5 // Maximum linear jerk of the Chassis in m/s/s/s
                            })
                .withOutput(*chassis)
                .buildMotionProfileController();

std::shared_ptr<AsyncMotionProfileController> profileControllerm =
        AsyncMotionProfileControllerBuilder()
                .withLimits({
                                    0.55, // Maximum linear velocity of the Chassis in m/s
                                    1, // Maximum linear acceleration of the Chassis in m/s/s
                                    5 // Maximum linear jerk of the Chassis in m/s/s/s
                            })
                .withOutput(*chassis)
                .buildMotionProfileController();

std::shared_ptr<AsyncMotionProfileController> profileController =
        AsyncMotionProfileControllerBuilder()
                .withLimits({
                                    0.75, // Maximum linear velocity of the Chassis in m/s
                                    1, // Maximum linear acceleration of the Chassis in m/s/s
                                    5 // Maximum linear jerk of the Chassis in m/s/s/s
                            })
                .withOutput(*chassis)
                .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
        AsyncPosControllerBuilder()
                .withMotor(18) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
        AsyncPosControllerBuilder()
                .withMotor(11) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> tipcontroller =
        AsyncPosControllerBuilder()
                .withMotor(15)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> intakecontroller =
        AsyncPosControllerBuilder()
                .withMotor(17)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

void skills(){

////    back up into mobile goal
//    tipcontroller->setTarget(2650);
//    profileControllers->generatePath({{0_in, 0_in, 0_deg}, {17_in, 0_in, 0_deg}}, "Z");
//    profileControllers->setTarget("Z", true);
//    profileControllers->waitUntilSettled();
//    tipcontroller->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//
//    //make arc and turn to get into open space and drive to neutral mobile goal
//    moveArc(p/3, 32, 15, true, false,100, 0);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "Y");
//    PIDTurnAbs(-1.95, 1);
//    profileController->setTarget("Y");
//    profileController->waitUntilSettled();
//
//    //clamp down on yellow and lift arm, so it doesn't interfere with driving
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {30_in, -14_in, 30_deg}}, "B");
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(1000);
//    liftcontroller->waitUntilSettled();
//
//    //drive in to opposite platform, run intake, lift arm high enough and removes paths
//    profileControllerm->setTarget("B");
//    liftcontroller->setTarget(3500);
//    intakecontroller->setTarget(10000);
//    liftcontroller->waitUntilSettled();
//    profileControllerm->waitUntilSettled();
//    profileControllers->removePath("Z");
//    profileController->removePath("Y");
//
//    //lower arm and open jaw to release
//    liftcontroller->setTarget(2300);
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//
//    //back up a little to synchronise arm up then back up again to synchronise arm down move forwards again to move rings
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {7_in, 0_in, 0_deg}}, "C");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {22_in, 0_in, 0_deg}}, "Y");
//    profileController->setTarget("C", true);
//    liftcontroller->setTarget(2800);
//    profileController->waitUntilSettled();
//    profileController->setTarget("C", true);
//    liftcontroller->setTarget(1000);
//    profileController->waitUntilSettled();
//    profileController->setTarget("C");
//    profileController->waitUntilSettled();
//
//    //make 90 turn and move to intake rings and 90 again and to move to yellow goal
//    PIDTurnAbs(-p, 2);
//    PIDTurnAbs(-p, 2);
//    pros::lcd::set_text(0, std::to_string(globalPos[2]));
////    PIDMove(90, -p, 1);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {29_in, 0_in, 0_deg}}, "Z");
//    profileController->setTarget("Z");
//    intakecontroller->setTarget(0);
//    profileController->waitUntilSettled();
//    profileController->setTarget("C", true);
//    profileController->waitUntilSettled();
//    PIDTurnAbs(-(3*p)/2, 2);
//    PIDTurnAbs(-(3*p)/2, 2);
//    tipcontroller->setTarget(2850);
//    tipcontroller->waitUntilSettled();
//    profileController->setTarget("Y");
//    liftcontroller->setTarget(50);
//    profileController->waitUntilSettled();
//
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(1000);
//    liftcontroller->waitUntilSettled();
//    profileControllers->removePath("Z");
//    profileController->removePath("Y");
//
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {26_in, -27_in, 0_deg}}, "D");
//    profileControllerm->setTarget("D");
//    liftcontroller->setTarget(3500);
//    intakecontroller->setTarget(10000);
//    liftcontroller->waitUntilSettled();
//    profileControllerm->waitUntilSettled();
//    liftcontroller->setTarget(2600);
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//
//    moveArc(1.1, 32, 10, false, true, 127);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {25_in, 0_in, 0_deg}}, "X");
//    PIDTurnAbs(-7, 1);
//    profileController->setTarget("X", true);
//    profileController->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {45_in, 8_in, 0_deg}}, "L");
//    liftcontroller->setTarget(100);
//    profileControllerm->setTarget("L");
//    profileControllerm->waitUntilSettled();
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(1000);
//    liftcontroller->waitUntilSettled();
//
//    PIDTurnAbs(-7.81, 2);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {35_in, 0_in, 0_deg}}, "O");
//    profileController->setTarget("O");
//    liftcontroller->setTarget(2600);
//    profileController->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//
//    profileController->setTarget("C", true);
//    profileController->waitUntilSettled();
//    PIDTurnAbs(-9.39, 1);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {25_in, 0_in, 0_deg}}, "M");
//    profileController->setTarget("M");
//    tipcontroller->setTarget(3000);
//    liftcontroller->setTarget(100);
//    profileController->waitUntilSettled();
//    jawcontroller->setTarget(-260);
//    tipcontroller->setTarget(100);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(3400);
//    liftcontroller->waitUntilSettled();
//    PIDTurnAbs(-6.4, 1);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {10_in, 0_in, 0_deg}}, "N");
//    profileController->setTarget("N");
//    liftcontroller->setTarget(2600);
//    profileController->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//    jawcontroller->waitUntilSettled();
//
////    profileController->generatePath({{0_in, 0_in, 0_deg}, {50_in, 20_in, -90_deg}}, "X");
////    profileController->setTarget("X", true);
////    liftcontroller->setTarget(2800);
}

void LRT(){
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {35_in, 0_in, 0_deg}}, "Z");
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "X");
//
//    profileController->setTarget("Z");
//    profileController->waitUntilSettled();
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(500);
//
//    moveHoldFork = false;
//    tipcontroller->setTarget(3150);
//    tipcontroller->waitUntilSettled();
//    moveHoldFork = true;
//
//    moveArc(1.3, 30, 25, true, true,127);
//
//    moveHoldFork = false;
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//    moveHoldFork = true;
//    setDrive(-100, 100);
//    pros::delay(400);
//    setDrive(0, 0);
//    pros::delay(500);
//
//    jawcontroller->setTarget(-20);
//    jawcontroller->waitUntilSettled();
//    PIDTurnAbs(1.1);
//    profileControllerm->setTarget("X");
//    intakecontroller->setTarget(5000);
//    liftcontroller->setTarget(50);
//    profileControllerm->waitUntilSettled();
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(500);
//    liftcontroller->waitUntilSettled();
//
//    setDrive(-100, 100);
//    PIDTurnAbs(3*p/4);
//    profileController->setTarget("Z");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("Y");
//    profileController->waitUntilSettled();
//
//    moveHoldFork = false;
//    tipcontroller->setTarget(3100);
//    tipcontroller->waitUntilSettled();
//    moveHoldFork = true;
}

