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

pros::Motor FLeft(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FRight(-3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Imu inertial_sensor (10);
std::shared_ptr<ChassisController> chassis =
		 ChassisControllerBuilder()
     .withMotors({-3,-12}, {4,5}) // left motor is 1, right motor is 2 (reversed)
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
    .withMotor(2) // lift motor port 3
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
profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
profileController->setTarget("A");
jawcontroller->setTarget(-400);
profileController->waitUntilSettled();
jawcontroller->setTarget(-500);
profileController->generatePath({{0_in, 0_in, 0_deg},{-30_in, 0_in, 0_deg}}, "B");
profileController->setTarget("B", true);
profileController->waitUntilSettled();
backintakecontroller->setTarget(2000);
backintakecontroller->waitUntilSettled();
leftsidecontroller->setTarget(FLeft.get_position()-2000);
rightsidecontroller->setTarget(FRight.get_position()+2000);
leftsidecontroller->waitUntilSettled();

// profileController->generatePath({{0_in, 0_in, 0_deg}, {-10_in, 0_in, 0_deg}}, "D");
// profileController->waitUntilSettled();
// profileController->setTarget("D", true);
pros::delay(1000);



//left side
//profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
// profileController->setTarget("A");
// profileController->waitUntilSettled();
// jawcontroller->setTarget(-400);
// jawcontroller->waitUntilSettled();
// profileController->generatePath({{0_in, 0_in, 0_deg},{-40_in, 0_in, 0_deg}}, "B");
// profileController->setTarget("B", true);
// profileController->waitUntilSettled();
//inertial_turn(270);
//
//
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
