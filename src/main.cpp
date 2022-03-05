#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
pros::Motor BLeft(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BRight(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FLeft(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FRight(3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Imu inertial_sensor (10);

std::shared_ptr<ChassisController> chassis =
		 ChassisControllerBuilder()
     .withMotors({12, 3}, {4,5}) // left motor is 1, right motor is 2 (reversed)
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

std::shared_ptr<AsyncMotionProfileController> profileController =
           AsyncMotionProfileControllerBuilder()
             .withLimits({
               1, // Maximum linear velocity of the Chassis in m/s
               1, // Maximum linear acceleration of the Chassis in m/s/s
               2.5 // Maximum linear jerk of the Chassis in m/s/s/s
             })
             .withOutput(chassis)
             .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
  AsyncPosControllerBuilder()
    .withMotor(7) // lift motor port 3
    //        .withGains({liftkP, liftkI, liftkD})
    .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
                 AsyncPosControllerBuilder()
                     .withMotor(6) // lift motor port 3
             //        .withGains({liftkP, liftkI, liftkD})
                     .build();
std::shared_ptr<AsyncPositionController<double, double>> backintakecontroller =
										                 AsyncPosControllerBuilder()
										                      .withMotor(1) // lift motor port 3
										              //        .withGains({liftkP, liftkI, liftkD})
										                      .build();
std::shared_ptr<AsyncPositionController<double, double>> rightsidecontroller =
							AsyncPosControllerBuilder()
					.withMotor({-3,-12}) // lift motor port 3
					.build();

std::shared_ptr<AsyncPositionController<double, double>> leftsidecontroller =
											 AsyncPosControllerBuilder()
															                      .withMotor({4,5}) // lift motor port 3
															              //        .withGains({liftkP, liftkI, liftkD})
															                      .build();



	void inertial_turn(int degrees) {
																											double x = inertial_sensor.get_heading();
																											double error = 0.769;
																											if (abs(degrees)>= 180) {
																												error = 0.905;
																											}

																											if (degrees > 0) {
																												while (inertial_sensor.get_heading() < x + degrees*error) {
																													double abc = inertial_sensor.get_heading();
																													pros::lcd::set_text(3, std::to_string(abc));
																													pros::lcd::set_text(4,"1st one");
																													FLeft.move(60);
																													BLeft.move(60);
																													FRight.move(-60);
																													BRight.move(-60);
																													pros::delay(10);
																													}
																													FLeft.move(0);
																													BLeft.move(0);
																													FRight.move(0);
																													BRight.move(0);
																												}

																											else {
																												double y = 360 - abs(degrees*error);
																												inertial_sensor.set_heading(359);
																												while (inertial_sensor.get_heading() > (y)) {
																													double abc = inertial_sensor.get_heading();
																													pros::lcd::set_text(3, std::to_string(abc));
																													pros::lcd::set_text(5,"2nd one");
																													FLeft.move(-60);
																													BLeft.move(-60);
																													FRight.move(60);
																													BRight.move(60);
																													pros::delay(10);
																													}
																													FLeft.move(0);
																													BLeft.move(0);
																													FRight.move(0);
																													BRight.move(0);
																											}
																										}
// void inertial_turn(int degrees) {
// inertial_sensor.reset();
// if (degrees > 0) {
// 										 		while (abs(inertial_sensor.get_heading()) < abs(degrees-2*0.98)) {
// 										 			FLeft.set_voltage_limit(-80);
// 										 			BLeft.set_voltage_limit(-80);
// 										 			FRight.set_voltage_limit(80);
// 										 			FRight.set_voltage_limit(80);
// 										 		}
// 										 	}
// 										 	else {
// 										 	while (abs(inertial_sensor.get_heading()) < abs(degrees-2*0.98)) {
// 										 		FLeft.set_voltage_limit(80);
// 										 		BLeft.set_voltage_limit(80);
// 										 		FRight.set_voltage_limit(-80);
// 										 		FRight.set_voltage_limit(-80);
// 										 	}
// 										 	}
// 										 		FLeft.set_voltage_limit(0);
// 										 		BLeft.set_voltage_limit(0);
// 										 		FRight.set_voltage_limit(0);
										 	// }
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(2, "Hello PROS User!");
	pros::lcd::set_text(1, "Hello PROS User!");

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
 * on the LCD.-
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
//right side
profileController->generatePath({{0_in, 0_in, 0_deg}, {39_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
jawcontroller->setTarget(-400);
profileController->waitUntilSettled();
jawcontroller->setTarget(-550);
profileController->generatePath({{0_in, 0_in, 0_deg},{-25_in, 0_in, 0_deg}}, "B");
profileController->setTarget("B", true);
profileController->waitUntilSettled();
backintakecontroller->setTarget(1900);
liftcontroller->setTarget(400);
inertial_turn(-125);
profileController->generatePath({{0_in, 0_in, 0_deg},{-20_in, 0_in, 0_deg}}, "C");
profileController->setTarget("C", true);
profileController->waitUntilSettled();
liftcontroller->setTarget(-300);
backintakecontroller->setTarget(1300);
profileController->generatePath({{0_in, 0_in, 0_deg},{15_in, 0_in, 0_deg}}, "D");
profileController->setTarget("D");
profileController->waitUntilSettled();
backintakecontroller->setTarget(1900);
pros::delay(4000);



//autos start
	//drive backwards and drop 3 rings and then pick up goal
	// profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
	// profileController->setTarget("A", "true");
	// profileController->waitUntilSettled();
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
// 	inertial_turn(90);
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
// 	inertial_turn(-90)
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
// inertial_turn(-50);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A", "true");
// forkcontroller->setTarget(1850);
// profileController->waitUntilSettled();
// forkcontroller->setTarget(1000);
//
// //90 turn to face 3rd neutral goal
// inertial_turn(70);
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
// inertial_turn(90);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// forkcontroller->setTarget(1850);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {8_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// jawcontroller->setTarget(-800);
// liftcontroller->setTarget(3500);
// liftcontroller->waitUntilSettled();
// inertial_turn(90);
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
// inertial_turn(90);
// liftcontroller->setTarget(10);
// profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// jawcontroller->setTarget(-800);
// inertial_turn(90);
// leftsidecontroller->waitUntilSettled();
// rightsidecontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// inertial_turn(90);
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
}
// Chassis Controller - lets us drive the robot around with open- or closed-loop control


void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(-19);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);
	}
}
