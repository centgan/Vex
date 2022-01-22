#include "main.h"
#include "fstream"

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
                                    0.5, // Maximum linear velocity of the Chassis in m/s
                                    1, // Maximum linear acceleration of the Chassis in m/s/s
                                    2.5 // Maximum linear jerk of the Chassis in m/s/s/s
                            })
                .withOutput(*chassis)
                .buildMotionProfileController();

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
    0.6, // Maximum linear velocity of the Chassis in m/s
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
    Fork.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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

void autonomous() {
//    std::string h = move(360, 75);
    pros::Task positionService(position, (void*)"Claw");
    double gearing = (double)left.getGearing();
//    double *pos = position();
//    Sensors_reset();

    tipcontroller->setMaxVelocity(150);
    tipcontroller->setTarget(3000);
    profileControllers->generatePath({{0_in, 0_in, 0_deg}, {17_in, 0_in, 0_deg}}, "Z");
    profileControllers->setTarget("Z", true);
    profileControllers->waitUntilSettled();
    tipcontroller->waitUntilSettled();
    tipcontroller->setTarget(2000);
    tipcontroller->waitUntilSettled();

    moveArc(p/5, 32, 15, true, 100, 0);
    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "Y");
    PIDTurnAbs(-1.88);
    profileController->setTarget("Y");
    profileController->waitUntilSettled();

    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {20_in, -28_in, 0_deg}}, "B");
    jawcontroller->setTarget(-260);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(500);
    liftcontroller->waitUntilSettled();
    PIDTurnAbs(-p/2);
    pros::lcd::set_text(0, std::to_string(globalPos[2]));
    profileControllerm->setTarget("B");
    liftcontroller->setTarget(3500);
    liftcontroller->waitUntilSettled();
    profileControllerm->waitUntilSettled();
    profileControllers->removePath("Z");
    profileController->removePath("Y");

    liftcontroller->setTarget(2400);
    liftcontroller->waitUntilSettled();
    jawcontroller->setTarget(-20);
    jawcontroller->waitUntilSettled();

    profileController->generatePath({{0_in, 0_in, 0_deg}, {7_in, 0_in, 0_deg}}, "C");
    profileController->generatePath({{0_in, 0_in, 0_deg}, {20_in, 0_in, 0_deg}}, "Z");
    profileController->generatePath({{0_in, 0_in, 0_deg}, {22_in, 0_in, 0_deg}}, "Y");
    profileController->generatePath({{0_in, 0_in, 0_deg}, {19_in, -18_in, 90_deg}}, "D");
    profileController->setTarget("C", true);
    profileController->waitUntilSettled();
    profileController->setTarget("C", true);
    liftcontroller->setTarget(600);
    profileController->waitUntilSettled();
    profileController->setTarget("C");
    profileController->waitUntilSettled();
    liftcontroller->waitUntilSettled();
    PIDTurnAbs(-p);
    pros::lcd::set_text(0, std::to_string(globalPos[2]));
    tipcontroller->setTarget(3100);
    tipcontroller->waitUntilSettled();
    profileController->setTarget("Z");
    intakecontroller->setTarget(10000);
    profileController->waitUntilSettled();
    PIDTurnAbs(-(3*p)/2);
    profileController->setTarget("Y");
    liftcontroller->setTarget(50);
    profileController->waitUntilSettled();
//    tipcontroller->setTarget(3100);
//    tipcontroller->waitUntilSettled();
//    profileController->setTarget("D");
//    liftcontroller->setTarget(50);
//    profileController->waitUntilSettled();
//
//    profileController->removePath("C");
//    profileController->removePath("D");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {10_in, -22_in, 90_deg}}, "E");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {18_in, 22_in, 0_deg}}, "F");
//
    jawcontroller->setTarget(-260);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(500);
    liftcontroller->waitUntilSettled();
//
//    PIDTurnAbs(-(3*p)/2);
//    PIDTurnAbs(p/2);
//    profileController->setTarget("E", true);
//    profileController->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//    profileController->setTarget("E");

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
//    pros::Task holdServiceClaw(holdClaw, (void*)"Claw");
//    pros::Task holdServiceFork(holdFork, (void*)"Fork");
    pros::Task holdService(holdFork, (void*)"Sub");
    while (true){
        drivemotors();
        liftArm();
        moveSub();
        pros::lcd::set_text(4, "bum");
        pros::delay(10);
    }

//    while(true){
//        double *pos = position();
//        pros::lcd::set_text(0, std::to_string(pos[0]));
//        pros::lcd::set_text(1, std::to_string(pos[1]));
//        pros::lcd::set_text(2, std::to_string(pos[2]));
//        drivemotors();
//        liftArm();
//        moveJaw();
//        moveTip();
//        drive->getModel()->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
//                                  controller.getAnalog(okapi::ControllerAnalog::leftX));
//        ControllerButton armUpButton(ControllerDigital::R1);
//        ControllerButton armDownButton(ControllerDigital::R2);
//        ControllerButton clawUp(ControllerDigital::R1);
//        ControllerButton clawDown(ControllerDigital::R2);
//        ControllerButton tipup(ControllerDigital::L2);
//        ControllerButton tipdown(ControllerDigital::L1);
//
//        pros::delay(10);
//    }
}

