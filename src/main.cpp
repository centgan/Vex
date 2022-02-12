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
//    pros::Motor tip (17, pros::E_MOTOR_GEARSET_18, false);
    DLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Fork.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task positionService(position, (void*)"Od");
//    pros::Task holdServiceFork(holdFork, (void*)"Fork");

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
    Sensors_reset();

    //back up into mobile goal
    tipcontroller->setTarget(2650);
    profileControllers->generatePath({{0_in, 0_in, 0_deg}, {17_in, 0_in, 0_deg}}, "Z");
    profileControllers->setTarget("Z", true);
    profileControllers->waitUntilSettled();
    tipcontroller->waitUntilSettled();
    tipcontroller->setTarget(2000);
    tipcontroller->waitUntilSettled();

    //make arc and turn to get into open space and drive to neutral mobile goal
    moveArc(p/3, 32, 15, true, false,100, 0);
    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "Y");
    PIDTurnAbs(-1.95, 1);
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
//    pros::Task holdService(holdFork, (void*)"Sub");
    while (true){
//        pros::lcd::set_text(2, std::to_string(encoder_right.get_value()));
//        pros::lcd::set_text(3, std::to_string(encoder_left.get_value()));
//        pros::lcd::set_text(4, std::to_string(encoder_rear.get_value()));
        pros::lcd::set_text(5, std::to_string(globalPos[0]));
        pros::lcd::set_text(6, std::to_string(globalPos[1]));
        pros::lcd::set_text(7, std::to_string(globalPos[2]));
        drivemotors();
        liftArm();
        moveSub();
//        pros::lcd::set_text(4, "bum");
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

