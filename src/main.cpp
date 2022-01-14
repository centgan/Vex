#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//MotorGroup left({13, 11});
//MotorGroup right({-18, 20});
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;
const double p = 2 * acos(0.0);

MotorGroup left({1, 11});
MotorGroup right({-19, 20});
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

std::shared_ptr<AsyncMotionProfileController> profileControllerf =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
    0.5, // Maximum linear velocity of the Chassis in m/s
    1, // Maximum linear acceleration of the Chassis in m/s/s
    5 // Maximum linear jerk of the Chassis in m/s/s/s
        })
        .withOutput(*chassis)
        .buildMotionProfileController();

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
    0.25, // Maximum linear velocity of the Chassis in m/s
    0.5, // Maximum linear acceleration of the Chassis in m/s/s
    2.5 // Maximum linear jerk of the Chassis in m/s/s/s
        })
        .withOutput(*chassis)
        .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
    AsyncPosControllerBuilder()
        .withMotor(12) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
        .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
    AsyncPosControllerBuilder()
        .withMotor(17) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
        .build();

std::shared_ptr<AsyncPositionController<double, double>> tipcontroller =
        AsyncPosControllerBuilder()
                .withMotor(15) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
	pros::lcd::initialize();
//    arm.setBrakeMode(AbstractMotor::brakeMode::hold);
//    jaw.setBrakeMode(AbstractMotor::brakeMode::hold);
//    tip.setBrakeMode(AbstractMotor::brakeMode::hold);
//    arm.setGearing(AbstractMotor::gearset::red);
////    jaw.setGearing(AbstractMotor::gearset::red);
//    tip.setGearing(AbstractMotor::gearset::red);
    DLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Fork.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    autonomous();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

struct MotorValues{
    int left, right;
};

typedef struct MotorValues Struct;

Struct calcMotorPower(float radius, bool dir, float maxPower){
    Struct motor;
    float d = radius * 2;
    float ratio = (d + 14.5)/(d - 14.5);

    if (dir){
        motor.left = maxPower;
        motor.right = maxPower/ratio;
    } else{
        motor.right = maxPower;
        motor.left = maxPower/ratio;
    }
    return motor;
}

float calc(float length, float height){
    float r = ((length)*(length)/(8 * height)) + (height/2);
    return r;
}

void autonomous() {
//    std::string h = move(360, 75);
    double gearing = (double)left.getGearing();

    Struct res;

    float rad = calc(36, 10);
    res = calcMotorPower(rad, true, 127);
    while(true){
        pros::lcd::set_text(0, std::to_string(rad));
        pros::lcd::set_text(1, std::to_string(res.left));
        pros::lcd::set_text(2, std::to_string(res.right));
        pros::delay(10);
//        setDrive(res.left, res.right);
    }

//    Sensors_reset();
//    tipcontroller->setTarget(3000);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {16_in, 0_in, 0_deg}}, "Z");
//    profileController->setTarget("Z", true);
//    profileController->waitUntilSettled();
//    tipcontroller->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//
//    PIDTurn(-p/2);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
//    profileController->setTarget("A");
//    profileController->waitUntilSettled();
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {29_in, -26_in, 30_deg}}, "B");
//    profileController->waitUntilSettled();
//
//    jawcontroller->setTarget(-250);
//    jawcontroller->waitUntilSettled();
//    profileController->setTarget("B");
//    liftcontroller->setTarget(3500);
//    profileController->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    profileController->removePath("A");
//    profileController->removePath("B");
//
//    liftcontroller->setTarget(2500);
//    liftcontroller->waitUntilSettled();
//    jawcontroller->setTarget(-20);
//    jawcontroller->waitUntilSettled();
//
//    pros::lcd::set_text(7, "bum");
//    PIDMove(-10);
//    PIDTurn(p/6);





//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "C");
//    liftcontroller->setTarget(-400);
//    jawcontroller->setTarget(1000);
//    profileController->setTarget("C", true);
//    liftcontroller->setTarget(-1900);


//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
//    profileController->setTarget("A");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 28_in, 0_deg}}, "B");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("B", true); // true means to use the path in reverse
//    profileController->removePath("A");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "C");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("C");
//    profileController->removePath("B");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {24_in, 0_in, 0_deg}}, "D");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("D",true); // true means to use the path in reverse
//    profileController->removePath("C");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "E");
//    profileController->waitUntilSettled();
//
//    chassis->setMaxVelocity(gearing*.25); // set the turn to be slower for more accuracy
//    chassis->turnAngle(-270_deg);
//    chassis->setMaxVelocity(gearing); // set veolcity back to full cartride value
//
//    profileController->setTarget("E");
//    profileController->waitUntilSettled();
//    profileController->removePath("E");

//    profileController->generatePath({{0_in, 0_in, 0_deg}, {24_in, 0_in, 0_deg}}, "C");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("C");
//    profileController->removePath("B");

//    chassis->setState({0_in, 0_in, 0_deg});
////    chassis->setMaxVelocity(150);
////    chassis->moveDistance(24_in);
//    chassis->driveToPoint({0_ft, 2_ft});
////    chassis->waitUntilSettled();
//    chassis->turnAngle(45_deg);
//    chassis->moveDistance(4_in);
//    chassis->turnAngle(-45_deg);
//    chassis->moveDistance(1_ft);
//    chassis->

//    chassis->driveToPoint({1_ft, 1_ft});
//    chassis->turnAngle(-45_deg);
//    chassis->stop();

//    drive->stop();

//    chassis->setState({0_in, 0_in, 0_deg});
//    chassis->turnAngle(45_deg);
//    chassis->moveDistance(1.4_ft);
//    Sensors_reset();
//    std::string moveVal = move(416, 90);
//    pros::lcd::set_text(0, moveVal);
////    std::string moveVal2 = move(5, 127);
////    pros::lcd::set_text(0, moveVal2);
//    pros::lcd::set_text(0, "hello");
//    pros::delay(50);
//    std::string turnVal = turn(1.5, 50);
//    pros::lcd::set_text(0, turnVal);
//    double *pos = position();
//    pros::lcd::set_text(1, std::to_string(pos[0]));
//    pros::lcd::set_text(2, std::to_string(pos[1]));
//    pros::lcd::set_text(3, std::to_string(pos[2]));

}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
//    Sensors_reset();
//    pros::Task holdServiceClaw(holdClaw, (void*)"Claw");
//    pros::Task holdServiceFork(holdFork, (void*)"Fork");
    pros::Task holdService(holdFork, (void*)"Sub");
    //    PIDMove(25);
//    PIDTurn( p/2);
    while (true){
        drivemotors();
        liftArm();
        moveSub();
        pros::lcd::set_text(2, std::to_string(claw.get_position()));
        pros::delay(10);
    }

//    while(true){
////        double *pos = position();
////        pros::lcd::set_text(0, std::to_string(pos[0]));
////        pros::lcd::set_text(1, std::to_string(pos[1]));
////        pros::lcd::set_text(2, std::to_string(pos[2]));
//
//        drivemotors();
//        liftArm();
//        moveJaw();
//        moveTip();
//
////        drive->getModel()->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
////                                  controller.getAnalog(okapi::ControllerAnalog::leftX));
////        ControllerButton armUpButton(ControllerDigital::R1);
////        ControllerButton armDownButton(ControllerDigital::R2);
////        ControllerButton clawUp(ControllerDigital::R1);
////        ControllerButton clawDown(ControllerDigital::R2);
////        ControllerButton tipup(ControllerDigital::L2);
////        ControllerButton tipdown(ControllerDigital::L1);
//
//        pros::delay(10);
//    }
}

