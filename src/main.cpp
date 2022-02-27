#include "main.h"

// pros::Motor FLeft(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor FRight(10, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor BLeft(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor BRight(14, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor FLeft(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FRight(3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BLeft(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BRight(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor Intake(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Lift(6, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Claw(2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Fork(1, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Imu inertial_sensor(10);


pros::Controller Master (pros::E_CONTROLLER_MASTER);
// pros::ADIEncoder encoder_right(7, 8, false);
// pros::ADIEncoder encoder_left(5, 6, true);
// pros::ADIEncoder encoder_rear(3, 4, true);

std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
		 .withMotors({4,5}, {3,12}) // left motor is 1, right motor is 2 (reversed)
		 .withGains(
						{0.0008, 0.0004, 0.00005}, // distance controller gains 0.00001
						{0.0008, 0, 0.00001}, // turn controller gains
						{0.0008, 0.000001, 0.00005}  // angle controller gains (helps drive straight)0.0001
		)
		.withSensors(
						ADIEncoder{'G', 'H'}, // left encoder in ADI ports A & B
						ADIEncoder{'E', 'F'},  // right encoder in ADI ports C & D (reversed)
						ADIEncoder{'C', 'D'}  // middle encoder in ADI ports E & F
		)
						// green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
						// 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
		.withDimensions(AbstractMotor::gearset::green, {{2.75_in, 9.5_in, 4_in, 2.75_in}, quadEncoderTPR})
		.withOdometry() // use the same scales as the chassis (above)
		.buildOdometry(); // build an odometry chassis

std::shared_ptr<AsyncMotionProfileController> profileController =
					 AsyncMotionProfileControllerBuilder()
						 .withLimits({
							 0.6, // Maximum linear velocity of the Chassis in m/s
							 1.0, // Maximum linear acceleration of the Chassis in m/s/s
							 2.5 // Maximum linear jerk of the Chassis in m/s/s/s
						 })
						 .withOutput(chassis)
						 .buildMotionProfileController();

	 std::shared_ptr<AsyncPositionController<double, double>> rightsidecontroller =
			 	AsyncPosControllerBuilder()
		 		.withMotor({3,11}) // lift motor port 3
						 		//        .withGains({liftkP, liftkI, liftkD})
				 		.build();

						std::shared_ptr<AsyncPositionController<double, double>> leftsidecontroller =
				 			 	AsyncPosControllerBuilder()
								// .withLimits({
								// 	0.25,
								// 	0.5,
								// 	2.5
																	//})
				 		 		.withMotor({4,5}) // lift motor port 3
				 						 		//        .withGains({liftkP, liftkI, liftkD})
				 				 		.build();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
	AsyncPosControllerBuilder()
		.withMotor(2) // lift motor port 3
		//        .withGains({liftkP, liftkI, liftkD})
		.build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
								 AsyncPosControllerBuilder()
										 .withMotor(6) // lift motor port 3
						 //        .withGains({liftkP, liftkI, liftkD})
										 .build();
std::shared_ptr<AsyncPositionController<double, double>> forkcontroller =
									AsyncPosControllerBuilder()
										.withMotor(1) // lift motor port 3
																 //        .withGains({liftkP, liftkI, liftkD})
										.build();
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

void inertial_turn(int degrees) {
	inertial_sensor.reset();
	if (degrees > 0) {
		while (abs(inertial_sensor.get_heading()) < abs(degrees-2)) {
			FLeft.set_voltage_limit(-80);
			BLeft.set_voltage_limit(-80);
			FRight.set_voltage_limit(80);
			FRight.set_voltage_limit(80);
		}
	}
	else {
	while (abs(inertial_sensor.get_heading()) < abs(degrees-2)) {
		FLeft.set_voltage_limit(80);
		BLeft.set_voltage_limit(80);
		FRight.set_voltage_limit(-80);
		FRight.set_voltage_limit(-80);
	}
	}
		FLeft.set_voltage_limit(0);
		BLeft.set_voltage_limit(0);
		FRight.set_voltage_limit(0);
		FRight.set_voltage_limit(0);
}





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "I'm hungry!");

	pros::lcd::register_btn1_cb(on_center_button);
	//autonomous();

	// pros::Motor FLeft(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor FRight(10, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor BLeft(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor BRight(14, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
	//
	// pros::Motor Lift(18, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor Claw(15, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor Fork(17, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
	//
	// FLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// FRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// BLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// BRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Fork.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
inertial_turn(90);




//autos start
	//drive backwards and drop 3 rings and then pick up goal
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
// 	profileController->setTarget("A", "true");
// 	profileController->waitUntilSettled();
// 	forkcontroller->setTarget(500);
// 	forkcontroller->waitUntilSettled();
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {5_in, 0_in, 0_deg}}, "A");
// 	pros::delay(250);
// 	forkcontroller->setTarget(1850);
// 	profileController->setTarget("A");
// 	forkcontroller->waitUntilSettled();
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {6_in, 0_in, 0_deg}}, "A");
// 	profileController->setTarget("A", "true");
// 	forkcontroller->setTarget(1000);
// 	forkcontroller->waitUntilSettled();
//
// //turn to face neutral goal
// 	leftsidecontroller->setTarget(FLeft.get_position()+1000);
// 	rightsidecontroller->setTarget(FRight.get_position() + 200);
// 	leftsidecontroller->waitUntilSettled();
//
// //pickup and stack first neutral goal
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
// 	profileController->setTarget("A");
// 	profileController->waitUntilSettled();
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, -24_in, 30_deg}}, "B");
// 	jawcontroller->setTarget(-350);
// 	jawcontroller->waitUntilSettled();
// 	profileController->setTarget("B");
// 	liftcontroller->setTarget(3500);
// 	liftcontroller->waitUntilSettled();
// 	profileController->waitUntilSettled();
// 	liftcontroller->setTarget(2600);
// 	liftcontroller->waitUntilSettled();
// 	jawcontroller->setTarget(-20);
//
// //drive back, 180, drop alliance gaol and push large neutral into red zone
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {6_in, 0_in, 0_deg}}, "A");
// 	profileController->setTarget("A", "true");
// 	profileController->waitUntilSettled();
// 	leftsidecontroller->setTarget(FLeft.get_position() - 800);
// 	rightsidecontroller->setTarget(FRight.get_position() + 800);
// 	leftsidecontroller->waitUntilSettled();
// 	rightsidecontroller->waitUntilSettled();
// 	forkcontroller->setTarget(1850);
// 	forkcontroller->waitUntilSettled();
//
// 	profileController->generatePath({{0_in, 0_in, 0_deg}, {45_in, 0_in, 0_deg}}, "A");
// 	profileController->setTarget("A", "true");
// 	pros::delay(300);
// 	forkcontroller->setTarget(10);
// 	profileController->waitUntilSettled();
//
// //90 turn to drive back and fork second alliance gaol
// leftsidecontroller->setTarget(FLeft.get_position() - 400);
// rightsidecontroller->setTarget(FRight.get_position() + 400);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A", "true");
// forkcontroller->setTarget(1850);
// profileController->waitUntilSettled();
// forkcontroller->setTarget(1000);
//
// //90 turn to face 3rd neutral goal
// leftsidecontroller->setTarget(FLeft.get_position() - 800);
// rightsidecontroller->setTarget(FRight.get_position() + 300);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
//
// //stack 3rd neutral goal
// profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, -22_in, 30_deg}}, "B");
// jawcontroller->setTarget(-350);
// jawcontroller->waitUntilSettled();
// profileController->setTarget("B");
// liftcontroller->setTarget(3500);
// liftcontroller->waitUntilSettled();
// profileController->waitUntilSettled();
// liftcontroller->setTarget(2600);
// liftcontroller->waitUntilSettled();
// jawcontroller->setTarget(-20);
//
// //turn 90, drop alliance, stack other alliance
// profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A", "true");
// profileController->waitUntilSettled();
// leftsidecontroller->setTarget(FLeft.get_position() - 400);
// rightsidecontroller->setTarget(FRight.get_position() + 400);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// forkcontroller->setTarget(1850);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// jawcontroller->setTarget(-800);
// liftcontroller->setTarget(3500);
// liftcontroller->waitUntilSettled();
// leftsidecontroller->setTarget(FLeft.get_position() + 400);
// rightsidecontroller->setTarget(FRight.get_position() - 400);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// liftcontroller->setTarget(2600);
// liftcontroller->waitUntilSettled();
// jawcontroller->setTarget(-20);
// pros::delay(100);
//
// //turn 90 grab red alliance turn 90 and drive to position to climb on red
// profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A", "true");
// profileController->waitUntilSettled();
// leftsidecontroller->setTarget(FLeft.get_position() + 400);
// rightsidecontroller->setTarget(FRight.get_position() - 400);
// liftcontroller->setTarget(10);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// jawcontroller->setTarget(-800);
// leftsidecontroller->setTarget(FLeft.get_position() - 400);
// rightsidecontroller->setTarget(FRight.get_position() + 400);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// leftsidecontroller->setTarget(FLeft.get_position() - 400);
// rightsidecontroller->setTarget(FRight.get_position() + 400);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
//
// //climb red platform with 1 alliance in claw
// liftcontroller->setTarget(2500);
// pros::delay(250);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// liftcontroller->setTarget(10);
// liftcontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();

//autos end


	// profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
	// profileController->setTarget("A");
	// profileController->waitUntilSettled();
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "B");
	// profileController->setTarget("B", "true");
	// profileController->waitUntilSettled();
	// liftcontroller->setTarget(2000);
	// liftcontroller->waitUntilSettled();
	// jawcontroller->setTarget(-700);
	// pros::delay(600);
	// liftcontroller->setTarget(5);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {29_in, 0_in, 0_deg}}, "C");
	// profileController->setTarget("C");
	// profileController->waitUntilSettled();
	// rightsidecontroller->setTarget(FRight.get_position() - 1130);
	// rightsidecontroller->waitUntilSettled();
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {60_in, 0_in, 0_deg}}, "D");
	// profileController->setTarget("D", "true");
	// profileController->waitUntilSettled();
	// leftsidecontroller->setTarget(FLeft.get_position() - 250);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "E");
	// profileController->setTarget("E", "true");
	// profileController->waitUntilSettled();

//15sec skills auton 2 mobile goals
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {57_in, 0_in, 0_deg}}, "A");
	// profileController->setTarget("A");
	// profileController->waitUntilSettled();
	// rightsidecontroller->setTarget(FRight.get_position() - 1130);
	// rightsidecontroller->waitUntilSettled();
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {60_in, 0_in, 0_deg}}, "B");
	// profileController->setTarget("B", "true");
	// profileController->waitUntilSettled();
	// leftsidecontroller->setTarget(FLeft.get_position() - 250);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "C");
	// profileController->setTarget("C", "true");
	// profileController->waitUntilSettled();


	// profileController->generatePath({{0_in, 0_in, 0_deg}, {5_in, 0_in, 0_deg}}, "B");
	// profileController->setTarget("B", "true");
	// profileController->waitUntilSettled();
	// leftsidecontroller->setTarget(-750);
	// pros::delay(1000);
	// leftsidecontroller->waitUntilSettled();
	// forkcontroller->setTarget(1850);
	// forkcontroller->waitUntilSettled();
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {10_in, 0_in, 0_deg}}, "C");
	// profileController->setTarget("C", "true");
	// profileController->waitUntilSettled();
	// forkcontroller->setTarget(5);
	// rightsidecontroller->setTarget(-500);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "D");
	// profileController->setTarget("D", "true");
	// profileController->waitUntilSettled();

//rings on 2 alliance goals
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
	// profileController->setTarget("A", "true");
	// profileController->waitUntilSettled();
	// forkcontroller->setTarget(500);
	// forkcontroller->waitUntilSettled();
	// leftsidecontroller->setTarget(1825);
	// pros::delay(300);
	// rightsidecontroller->setTarget(FRight.get_position() - 90);
	// leftsidecontroller->waitUntilSettled();
	//
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {67_in, 0_in, 0_deg}}, "B");
	// profileController->setTarget("B");
	// profileController->waitUntilSettled();
	// leftsidecontroller->setTarget(FLeft.get_position() + 1750);
	// leftsidecontroller->waitUntilSettled();
	// pros::delay(100);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {14_in, 0_in, 0_deg}}, "C");
	// profileController->setTarget("C");
	// profileController->waitUntilSettled();
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {3_in, 0_in, 0_deg}}, "D");
	// profileController->setTarget("D", "true");
	// profileController->waitUntilSettled();
	// liftcontroller->setTarget(1325);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {4_in, 0_in, 0_deg}}, "E");
	// profileController->setTarget("D");
	// profileController->waitUntilSettled();
	// jawcontroller->setTarget(-850);
	// forkcontroller->setTarget(5);
	// jawcontroller->waitUntilSettled();
	// liftcontroller->setTarget(5);
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {6_in, 0_in, 0_deg}}, "F");
	// profileController->setTarget("F", "true");
	// profileController->waitUntilSettled();
//autos end



	// pros::delay(500);
//	leftsidecontroller->setTarget(100);
	//leftsidecontroller->waitUntilSettled();


	// pros::delay(500);
	// FRight.move_absolute(100,100);
	// BRight.move_absolute(100,100);
	// FLeft.move_absolute(300,100);
	// BLeft.move_absolute(300,100);
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
	FLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Fork.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	double armPwr = 0;
	double ClawOpen = 0;
	double ClawClose = 0;
	double ForkOpen = 0;
	double ForkClose = 0;
	bool Intake01 = true;

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
			//Fork.move_absolute(double (1850), 127);
			forkcontroller->setTarget(1755);


		} else if (Master.get_digital(DIGITAL_L1)){
			//Fork.move_absolute(double (1000), 127);
			forkcontroller->setTarget(1300);


		}
		else if (Master.get_digital(DIGITAL_UP)){
			//Fork.move_absolute(double (10), 127);
			forkcontroller->setTarget(15);

		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0){
			armPwr = -armPwr;
		}

		if (Master.get_digital(DIGITAL_A)) {
			if (Intake01 = true) {
				Intake01 = false;
			}
			else {
				Intake01 = true;
			}
}

	if (Lift.get_position() >= 500 && Intake01) {
			Intake.move_velocity(85);
		}
	else {
		Intake.move_velocity(0);
	}

		leftPower = Pwr - 0.6*Trn;
		rightPower = Pwr + 0.6*Trn;

		FLeft.move(rightPower);
		BLeft.move(rightPower);
		FRight.move(leftPower);
		BRight.move(leftPower);
		Lift.move(armPwr);
		pros::delay(20);
	}
}
