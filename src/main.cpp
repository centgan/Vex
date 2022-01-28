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
    pros::Task positionService(position, (void*)"Od");
//    double gearing = (double)left.getGearing();
//    double *pos = position();

    Sensors_reset();
    LRT();

//    profileController->generatePath({{0_in, 0_in, 0_deg}, {10_in, 5_in, 90_deg}}, "Z");
//    profileController->setTarget("Z", true);
//    profileController->waitUntilSettled();
//    PIDTurnAbs(-p/2);
//    PIDMove(5, -p/2, 1);
//    tipcontroller->waitUntilSettled();
//    tipcontroller->setTarget(2000);
//    tipcontroller->waitUntilSettled();
//    PIDTurnAbs(-p/2);
//    moveArc(p/2, 40, 30, false, true, 100);
//    PIDTurnAbs(-p);


//    moveArc(p/5, 32, 15, true, 100, 0);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, 0_in, 0_deg}}, "Y");
//    PIDTurnAbs(-1.85);
//    profileController->setTarget("Y");
//    profileController->waitUntilSettled();
//
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {20_in, -28_in, 0_deg}}, "B");
//    jawcontroller->setTarget(-260);
//    jawcontroller->waitUntilSettled();
//    liftcontroller->setTarget(500);
//    liftcontroller->waitUntilSettled();

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
        pros::lcd::set_text(7, std::to_string(globalPos[2]));
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

