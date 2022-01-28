#include "main.h"

MotorGroup left({1, 11});
MotorGroup right({18, 19});
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
                .withMotor(13) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
        AsyncPosControllerBuilder()
                .withMotor(17) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> tipcontroller =
        AsyncPosControllerBuilder()
                .withMotor(15)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> intakecontroller =
        AsyncPosControllerBuilder()
                .withMotor(14)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

void autoSkills(){
//    tipcontroller->setMaxVelocity(150);
//    tipcontroller->setTarget(3000);
//    profileControllers->generatePath({{0_in, 0_in, 0_deg}, {17_in, 0_in, 0_deg}}, "Z");
//    profileControllers->setTarget("Z", true);
//    profileControllers->waitUntilSettled();
//    tipcontroller->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//
//    moveArc(p/5, 32, 15, true, 100, 0);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "Y");
//    PIDTurnAbs(-1.87);
//    profileController->setTarget("Y");
//    profileController->waitUntilSettled();
//
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {20_in, -28_in, 0_deg}}, "B");
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(500);
//    liftcontroller->waitUntilSettled();
//    PIDTurnAbs(-p/2);
//    pros::lcd::set_text(0, std::to_string(globalPos[2]));
//    profileControllerm->setTarget("B");
//    liftcontroller->setTarget(3500);
//    liftcontroller->waitUntilSettled();
//    profileControllerm->waitUntilSettled();
//    profileControllers->removePath("Z");
//    profileController->removePath("Y");
//
//    liftcontroller->setTarget(2400);
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//    jawcontroller->waitUntilSettled();
//
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {7_in, 0_in, 0_deg}}, "C");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {20_in, 0_in, 0_deg}}, "Z");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {22_in, 0_in, 0_deg}}, "Y");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {19_in, -18_in, 90_deg}}, "D");
//    profileController->setTarget("C", true);
//    profileController->waitUntilSettled();
//    profileController->setTarget("C", true);
//    liftcontroller->setTarget(600);
//    profileController->waitUntilSettled();
//    profileController->setTarget("C");
//    profileController->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    PIDTurnAbs(-p);
//    pros::lcd::set_text(0, std::to_string(globalPos[2]));
//    tipcontroller->setTarget(3100);
//    tipcontroller->waitUntilSettled();
//    profileController->setTarget("Z");
//    intakecontroller->setTarget(10000);
//    profileController->waitUntilSettled();
//    PIDTurnAbs(-(3*p)/2);
//    profileController->setTarget("Y");
//    liftcontroller->setTarget(50);
//    profileController->waitUntilSettled();
//
////    tipcontroller->setTarget(3100);
////    tipcontroller->waitUntilSettled();
////    profileController->setTarget("D");
////    liftcontroller->setTarget(50);
////    profileController->waitUntilSettled();
////
////    profileController->removePath("C");
////    profileController->removePath("D");
////    profileController->generatePath({{0_in, 0_in, 0_deg}, {10_in, -22_in, 90_deg}}, "E");
////    profileController->generatePath({{0_in, 0_in, 0_deg}, {18_in, 22_in, 0_deg}}, "F");
////
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(500);
//    liftcontroller->waitUntilSettled();
////
////    PIDTurnAbs(-(3*p)/2);
////    PIDTurnAbs(p/2);
////    profileController->setTarget("E", true);
////    profileController->waitUntilSettled();
////    tipcontroller->setTarget(2000);
////    tipcontroller->waitUntilSettled();
////    profileController->setTarget("E");
}

void LRT(){
    profileController->generatePath({{0_in, 0_in, 0_deg}, {35_in, 0_in, 0_deg}}, "Z");
    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "X");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {20_in, 40_in, 0_deg}}, "Y");
    profileController->setTarget("Z");
    profileController->waitUntilSettled();
    jawcontroller->setTarget(-260);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(500);
    tipcontroller->setTarget(3150);
    tipcontroller->waitUntilSettled();
    moveArc(1.3, 30, 25, true, true,127);
    pros::delay(500);
    tipcontroller->setTarget(2000);
    tipcontroller->waitUntilSettled();
//    PIDTurnAbs(3*p/4);
    setDrive(-100, 100);
    pros::delay(400);
    setDrive(0, 0);
    jawcontroller->setTarget(-20);
    jawcontroller->waitUntilSettled();
    PIDTurnAbs(1.16);
    profileControllerm->setTarget("X");
    intakecontroller->setTarget(5000);
    liftcontroller->setTarget(50);
    profileControllerm->waitUntilSettled();
    jawcontroller->setTarget(-260);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(500);
    liftcontroller->waitUntilSettled();

    setDrive(-100, 100);
    PIDTurnAbs(7*p/8);
    profileController->setTarget("Z");
    profileController->waitUntilSettled();

    profileController->setTarget("Y");
    profileController->waitUntilSettled();

    tipcontroller->setTarget(3100);

}

