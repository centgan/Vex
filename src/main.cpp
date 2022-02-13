#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
	pros::Controller Master (pros::E_CONTROLLER_MASTER);
	pros::Motor FrntLeft (1, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor FrntRight (10, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor BckLeft (3, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor BckRight (6, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor Lift (2, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor Claw (7, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor Fork (4, pros::E_MOTOR_GEARSET_18, false);

	FrntLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FrntRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BckLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BckRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Fork.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
void autonomous() {}
	//drive backwards and drop 3 rings and then pick up goal
	profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A", "true");
	profileController->waitUntilSettled();
	forkcontroller->setTarget(500);
	forkcontroller->waitUntilSettled();
	profileController->generatePath({{0_in, 0_in, 0_deg}, {4_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A");
	forkcontroller->setTarget(1850);
	forkcontroller->waitUntilSettled();
	profileController->generatePath({{0_in, 0_in, 0_deg}, {5_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A", "true");
	forkcontroller->setTarget(1000);
	forkcontroller->waitUntilSettled();
	
//turn to face neutral goal
	leftsidecontroller->setTarget(FLeft.get_position()+1000);
	rightsidecontroller->setTarget(FRight.get_position() + 200);
	leftsidecontroller->waitUntilSettled();

//pickup and stack first neutral goal
	profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A");
	profileController->waitUntilSettled();
	profileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, -24_in, 30_deg}}, "B");
	jawcontroller->setTarget(-350);
	jawcontroller->waitUntilSettled();
	profileController->setTarget("B");
	liftcontroller->setTarget(3500);
	liftcontroller->waitUntilSettled();
	profileController->waitUntilSettled();
	liftcontroller->setTarget(2600);
	liftcontroller->waitUntilSettled();
	jawcontroller->setTarget(-20);

//drive back, 180, drop alliance gaol and push large neutral into red zone
	profileController->generatePath({{0_in, 0_in, 0_deg}, {6_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A", "true");
	profileController->waitUntilSettled();
	leftsidecontroller->setTarget(FLeft.get_position() - 800);
	rightsidecontroller->setTarget(FRight.get_position() + 800);
	leftsidecontroller->waitUntilSettled();
	rightsidecontroller->waitUntilSettled();
	forkcontroller->setTarget(1850);
	forkcontroller->waitUntilSettled();

	profileController->generatePath({{0_in, 0_in, 0_deg}, {45_in, 0_in, 0_deg}}, "A");
	profileController->setTarget("A", "true");
	pros::delay(300);
	forkcontroller->setTarget(10);
	profileController->waitUntilSettled();

//90 turn to drive back and fork second alliance gaol
leftsidecontroller->setTarget(FLeft.get_position() - 400);
rightsidecontroller->setTarget(FRight.get_position() + 400);
leftsidecontroller->waitUntilSettled();
rightsidecontroller->waitUntilSettled();
profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A", "true");
forkcontroller->setTarget(1850);
profileController->waitUntilSettled();
forkcontroller->setTarget(1000);

//90 turn to face 3rd neutral goal
leftsidecontroller->setTarget(FLeft.get_position() - 800);
rightsidecontroller->setTarget(FRight.get_position() + 300);
leftsidecontroller->waitUntilSettled();
rightsidecontroller->waitUntilSettled();

//stack 3rd neutral goal
profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
profileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, -22_in, 30_deg}}, "B");
jawcontroller->setTarget(-350);
jawcontroller->waitUntilSettled();
profileController->setTarget("B");
liftcontroller->setTarget(3500);
liftcontroller->waitUntilSettled();
profileController->waitUntilSettled();
liftcontroller->setTarget(2600);
liftcontroller->waitUntilSettled();
jawcontroller->setTarget(-20);

//turn 90, drop alliance, stack other alliance
profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A", "true");
profileController->waitUntilSettled();
leftsidecontroller->setTarget(FLeft.get_position() - 400);
rightsidecontroller->setTarget(FRight.get_position() + 400);
leftsidecontroller->waitUntilSettled();
rightsidecontroller->waitUntilSettled();
forkcontroller->setTarget(1850);
profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
jawcontroller->setTarget(-800);
liftcontroller->setTarget(3500);
liftcontroller->waitUntilSettled();
leftsidecontroller->setTarget(FLeft.get_position() + 400);
rightsidecontroller->setTarget(FRight.get_position() - 400);
profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
liftcontroller->setTarget(2600);
liftcontroller->waitUntilSettled();
jawcontroller->setTarget(-20);
pros::delay(100);

//turn 90 grab red alliance turn 90 and drive to position to climb on red
profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A", "true");
profileController->waitUntilSettled();
leftsidecontroller->setTarget(FLeft.get_position() + 400);
rightsidecontroller->setTarget(FRight.get_position() - 400);
liftcontroller->setTarget(10);
profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
jawcontroller->setTarget(-800);
leftsidecontroller->setTarget(FLeft.get_position() - 400);
rightsidecontroller->setTarget(FRight.get_position() + 400);
leftsidecontroller->waitUntilSettled();
rightsidecontroller->waitUntilSettled();
profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
leftsidecontroller->setTarget(FLeft.get_position() - 400);
rightsidecontroller->setTarget(FRight.get_position() + 400);
leftsidecontroller->waitUntilSettled();
rightsidecontroller->waitUntilSettled();

//climb red platform with 1 alliance in claw
liftcontroller->setTarget(2500);
pros::delay(250);
profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();
liftcontroller->setTarget(10);
liftcontroller->waitUntilSettled();
profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
profileController->waitUntilSettled();

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



	double armPwr = 0;
	double ClawOpen = 0;
	double ClawClose = 0;
	double ForkOpen = 0;
	double ForkClose = 0;


	double Pwr = 0;
	double Trn = 0;
	double leftPower = 0;
	double rightPower = 0;
	int deadband = 5;

	Fork.set_zero_position(0);
	while (true) {
		int x = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		int y = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		double armPos = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));


		double LjoyY = y/10.0;
		double LjoyX = x/10.0;
		double finalPWr = 0;

		if ( (y > deadband) || (x > deadband)) {

			if (y > 85) {

				Pwr = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 3.0;

			}

			else if (y > 55 && y <= 85){

				Pwr = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 10.0;

			}
			else {

				Pwr = 5*pow((1.0/5.5)*(LjoyY), 3.0) * 12.7;

			}

			if (x > 85) {

				Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 3.0;

			}

			else if (x > 55 && x <= 85){

				Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 10.0;

			}
			else {

				Trn = 5*pow((1.0/5.5)*(LjoyX), 3.0) * 12.7;

			}

		}
		else {
			Pwr = 0;
			Trn = 0;
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0){
			Pwr = -Pwr;
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0){
			Trn = -Trn;
		}

		if (armPos > deadband){

			if (armPos > 85) {

				armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 3.0) * 1;

			}

			else if (armPos > 55 && armPos <= 85){

				armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 10.0) * 1;

			}
			else {

				armPwr = (5*pow((1.0/5.5)*(armPos/10.0), 3.0) * 12.7) * 1;

			}
		}

		else {
			armPwr = 0;
		}

    // pros::Controller clawOpen(ControllerDigital::R1);
    // pros::Controller Master clawClose(ControllerDigital::R2);
    // pros::ControllerButton ForkOpen(ControllerDigital::L1);
    // ControllerButton ForkClose(pros::E_CONTROLLER_DIGITAL_L2);
		// Master.print (DIGITAL_R1);
		// Master.print (DIGITAL_R2);
		//while (Master.get_digital(DIGITAL_R1) or Master.get_digital(DIGITAL_R2)) {

			if (Master.get_digital(DIGITAL_R1)) {
				Claw.move_velocity(75);
				}
			else if (Master.get_digital(DIGITAL_R2)) {
				Claw.move_velocity(-75);
			}
			else {
			Claw.move_velocity(0);
		}

		if (Master.get_digital(DIGITAL_L2)){
			Fork.move_absolute(double (1850), 100);

		} else if (Master.get_digital(DIGITAL_L1)){
			Fork.move_absolute(double (1000), 100);

		}
		else if (Master.get_digital(DIGITAL_UP)){
			Fork.move_absolute(double (10), 100);
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0){
			armPwr = -armPwr;
		}

		leftPower = Pwr + 0.8*Trn;
		rightPower = Pwr - 0.8*Trn;

		FrntLeft.move(leftPower);
		BckLeft.move(leftPower);
		FrntRight.move(rightPower);
		BckRight.move(rightPower);
		Lift.move(armPwr);


		pros::lcd::set_text(1, std::to_string(Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)));
		// pros::lcd::set_text(2, std::to_string(y));
		// pros::lcd::set_text(3, std::to_string(Trn));
		// pros::lcd::set_text(4, std::to_string(x));
		// pros::lcd::set_text(5, std::to_string(LjoyY));
		// pros::lcd::set_text(6, std::to_string(Trn));
		pros::delay(20);
	}
}
