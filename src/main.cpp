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
	pros::lcd::initialize();

    Sensors_reset();

    DLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Fork.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(500);
    pros::Task positionService(position, (void*)"Od");
    pros::Task write(data, (void*)"da");
    pros::Task holdServiceFork(holdFork, (void*)"Fork");

//    autonomous();
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
std::vector<double> finalX;
std::vector<double> finalY;
std::vector<std::vector <double>> final(2, std::vector<double>(0,0));

void cubBezier(std::vector<double>& x, std::vector<double>& y){
    finalX.clear();
    finalY.clear();
    final.clear();
    double i = 0;
    while (i < 1.01){
        double P0 = pow((1-i), 2);
        double P1 = 2*(1-i)*(i);
        double P2 = pow(i, 2);

        double P0X = P0 * x[0];
        double P0Y = P0 * y[0];
        double P1X = P1 * x[1];
        double P1Y = P1 * y[1];
        double P2X = P2 * x[2];
        double P2Y = P2 * y[2];

        double totalX = P0X + P1X + P2X;
        double totalY = P0Y + P1Y + P2Y;
        finalX.push_back(totalX);
        finalY.push_back(totalY);

        i = i + 0.5;
    }

    std::ofstream output_fileX("/usd/Xcoods.txt");
    std::ostream_iterator<double> output_iteratorX(output_fileX, "\n");
    std::copy(finalX.begin(), finalX.end(), output_iteratorX);

    std::ofstream output_fileY("/usd/Ycoods.txt");
    std::ostream_iterator<double> output_iteratorY(output_fileY, "\n");
    std::copy(finalY.begin(), finalY.end(), output_iteratorY);

    final[0] = finalX;
    final[1] = finalY;
}

std::vector<std::vector<double>> quadBezier(std::vector<double> x, std::vector<double> y){
    std::vector<std::vector <double>> final;
    finalX.clear();
    finalY.clear();
    double i = 0;
    while (i < 1.01){
        double P0 = pow((1-i), 3);
        double P1 = 3*(pow(1-i, 2))*(i);
        double P2 = 3*pow(1-i, 2)*pow(i, 2);
        double P3 = pow(i, 3);

        double P0X = P0 * x[0];
        double P0Y = P0 * y[0];
        double P1X = P1 * x[1];
        double P1Y = P1 * y[1];
        double P2X = P2 * x[2];
        double P2Y = P2 * y[2];
        double P3X = P3 * x[3];
        double P3Y = P3 * y[3];

        double totalX = P0X + P1X + P2X + P3X;
        double totalY = P0Y + P1Y + P2Y + P3Y;
        finalX.push_back(totalX);
        finalY.push_back(totalY);

        i = i + 0.05;
    }

    std::ofstream output_fileX("/usd/Xcoods.txt");
    std::ostream_iterator<double> output_iteratorX(output_fileX, "\n");
    std::copy(finalX.begin(), finalX.end(), output_iteratorX);

    std::ofstream output_fileY("/usd/Ycoods.txt");
    std::ostream_iterator<double> output_iteratorY(output_fileY, "\n");
    std::copy(finalY.begin(), finalY.end(), output_iteratorY);

    final[0] = finalX;
    final[1] = finalY;
    return final;
}


//void sum(double x, double y){
//    while(true){
//        double v = 0.1;
//        double o = 0.1;
//        double d = 14;
//        setDrive()
//    }
//}


void autonomous() {
////    slot 1
//    new_right();
//    slot 2
    new_left();

////    slot 3
//    moveHoldFork = false;
//    while(globalPos[1] > -12){
//        if(Fork.get_position() > -1000){
//            Fork.move_voltage(-12000);
//        }else{
//            Fork.move_voltage(0);
//        }
//        setDrive(-50, -50);
//    }
//////    moveHoldFork = true;
//////    setDrive(0, 0);
//////    moveHoldFork = false;
//    while(Fork.get_position() < 1200){
//        Fork.move_voltage(12000);
//        lift.move_voltage(120000);
//    }
//    lift.move_voltage(0);
//    moveHoldFork = true;
//
//    setDrive(25, 25);
//    pros::delay(400);
//    setDrive(0, 0);
//
//    for(int i = 0; i < 2; i++){
//        while(globalPos[0] < 7){
//            setDrive(25, 25);
//            intake.move_voltage(-9600);
//        }
//        if (i != 1){
//            setDrive(-60, -60);
//            pros::delay(800);
//        }
//    }
//    setDrive(0, 0);
//    moveHoldFork = false;
//    while(Fork.get_position() > -2000){
//        Fork.move_voltage(-12000);
//    }
//    moveHoldFork = true;





//    MoveToPoint(-25, 10, p/2, 127, false);
//    MoveToPoint(-55, 0, -p/2, 127, false, false, true, 0, -1400, true, -10000);
//
//    while(claw.get_position() < -580){
//        claw.move_voltage(10000);
//    }
//    claw.move_voltage(0);
//
////    while(lift.get_position() < 500){
////        lift.move_voltage(12000);
////    }
////    lift.move_voltage(0);
//
////    MoveToPoint(-97, -40, p/2, 110, false, true, true, 2, 2400, false, 12000);
////    MoveToPoint(-97, -60, p/2, 110, false, true, true, 2, 2400, false, 12000);
////    setDrive(-50, 50);
////    pros::delay(1000);
////    MoveToPoint(-92, -35, p/2, 110, false);
////    PIDTurnAbs(-0.55);
////    while(globalPos[0] > -97){
////        setDrive(80, 80);
////        if(lift.get_position() < 2800){
////            lift.move_voltage(10000);
////        }else{
////            lift.move_voltage(0);
////        }
////    }
////    setDrive(0, 0);
////    MoveToPoint(-100, -35, p/2, 127, false);
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {20_in, -2_in, 30_deg}}, "B");
//    profileControllerm->setTarget("B");
//    intakecontroller->setTarget(10000);
//    liftcontroller->setTarget(3000);
//    profileControllerm->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    PIDTurnAbs(-1.35);
//    while(globalPos[0] < -108){
//        setDrive(80, 80);
//    }
//    while(claw.get_position() > -1200){
//        claw.move_voltage(-10000);
//    }
//    claw.move_voltage(0);
//
//    setDrive(-40, -100);
//    pros::delay(400);
//    while(globalPos[0] < -90){
//        setDrive(-30, -30);
//    }
//    setDrive(0, 0);
//
//    PIDTurnAbs(-p);
//    while(globalPos[1] > -67){
//        setDrive(40, 40);
//        intake.move_voltage(-10000);
//    }
//
//    PIDTurnAbs(-3*p/2);
//    while(lift.get_position() > 1000){
//        lift.move_voltage(-12000);
//        if(claw.get_position() > -1100){
//            claw.move_voltage(-10000);
//        }else{
//            claw.move_voltage(0);
//        }
//    }
//    while(globalPos[0] < -70){
//        setDrive(80, 80);
//        if(lift.get_position() > 0){
//            lift.move_voltage(-12000);
//        }else{
//            lift.move(0);
//        }
//    }
//    setDrive(0, 0);
//
//    while(claw.get_position() < -580){
//        claw.move_voltage(10000);
//    }
//    claw.move_voltage(0);
//
//
//    setDrive(-30, -100);
//    pros::delay(1250);
//    setDrive(0, 0);
//
////    MoveToPoint(-95, -40, p/2, 127, false, false, true, 2, 2400, false, 9000);
//    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {50_in, 30_in, 90_deg}}, "A");
//    profileControllerm->setTarget("A");
//    liftcontroller->setTarget(2900);
//    intakecontroller->setTarget(20000);
//    profileControllerm->waitUntilSettled();
//    liftcontroller->waitUntilSettled();
//    while(claw.get_position() > -1000){
//        claw.move_voltage(-10000);
//    }
//
//    setDrive(-100, -20);
//    pros::delay(550);
//    setDrive(0, 0);
//
//    MoveToPoint(-55, -50, p/2, 127, false, false, true, 2, 0, true, -10000);
//    while(claw.get_position() < -550){
//        claw.move_voltage(10000);
//    }
//
//// add multitask to pidturnabs
//    PIDTurnAbs(-p/2, true, 2, 2800, false, 12000);
//    MoveToPoint(-80, -30, p/2, 127, false);
//    MoveToPoint(-100, -30, p/2, 127, false, false, true, 1, -1200, true, 12000);
//    while(claw.get_position() > -1500){
//        claw.move_voltage(-10000);
//    }
//    double a = p-atan2(-75-globalPos[0], 20-globalPos[1]);
//    PIDTurnAbs(a);
//    while(globalPos[1] < 20){
//        setDrive(-80, -80);
//    }
//    while(Fork.get_position() > 0){
//        Fork.move_voltage(12000);
//    }
//    Fork.move_voltage(0);
//
//    MoveToPoint(-70, -30, p/2, 127, false);
//
//    while(claw.get_position() < -775){
//        claw.move_voltage(10000);
//    }



//// run intake
//    MoveToPoint(-70, -87, p/2, 127, false, true);
//    PIDTurnAbs(-3*p/2);
//    MoveToPoint(-55, -100, p/2, 127, false, false, true,2, 0, true, -10000);

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
    while (true){
//        pros::lcd::set_text(2, std::to_string(encoder_right.get_value()));
//        pros::lcd::set_text(3, std::to_string(encoder_left.get_value()));
//        pros::lcd::set_text(4, std::to_string(encoder_rear.get_value()));
//        pros::lcd::set_text(5, std::to_string(globalPos[0]));
//        pros::lcd::set_text(6, std::to_string(globalPos[1]));
//        pros::lcd::set_text(7, std::to_string(globalPos[2]));
        pros::lcd::set_text(0, std::to_string(Fork.get_position()));
        drivemotors();
        liftArm();
        moveSub();

        pros::delay(15);
    }
}

