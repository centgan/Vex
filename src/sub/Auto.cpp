#include "main.h"

MotorGroup left({14, 13});
MotorGroup right({15, 18});
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

std::shared_ptr<AsyncMotionProfileController> rprofileController =
        AsyncMotionProfileControllerBuilder()
                .withLimits({
                                    1, // Maximum linear velocity of the Chassis in m/s
                                    1, // Maximum linear acceleration of the Chassis in m/s/s
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
                .withMotor(6) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
        AsyncPosControllerBuilder()
                .withMotor(11) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> tipcontroller =
        AsyncPosControllerBuilder()
                .withMotor(17)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

std::shared_ptr<AsyncPositionController<double, double>> intakecontroller =
        AsyncPosControllerBuilder()
                .withMotor(9)
//        .withGains({liftkP, liftkI, liftkD})
                .build();

//slot 5
void skills(){
    tipcontroller->setTarget(3100);
    profileControllers->generatePath({{0_in, 0_in, 0_deg}, {19_in, 0_in, 0_deg}}, "Z");
    profileControllers->setTarget("Z", true);
    profileControllers->waitUntilSettled();
    tipcontroller->waitUntilSettled();
    tipcontroller->setTarget(2200);
    tipcontroller->waitUntilSettled();

    //make arc and turn to get into open space and drive to neutral mobile goal
    moveArc(p/3, 32, 15, true, false,127, 0);
    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "Y");
    pros::delay(200);
    PIDTurnAbs(-1.95);
    profileController->setTarget("Y");
    profileController->waitUntilSettled();

    //clamp down on yellow and lift arm, so it doesn't interfere with driving
    profileControllerm->generatePath({{0_in, 0_in, 0_deg}, {27_in, -7_in, 30_deg}}, "B");
    jawcontroller->setTarget(-240);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(1000);
    liftcontroller->waitUntilSettled();

    //drive in to opposite platform, run intake, lift arm high enough and removes paths
    profileControllerm->setTarget("B");
    liftcontroller->setTarget(3500);
    intakecontroller->setTarget(-10000);
    liftcontroller->waitUntilSettled();
    profileControllerm->waitUntilSettled();
    profileControllers->removePath("Z");
    profileController->removePath("Y");

    //lower arm and open jaw to release
    liftcontroller->setTarget(2300);
    liftcontroller->waitUntilSettled();
    jawcontroller->setTarget(-20);

    //back up a little to synchronise arm up then back up again to synchronise arm down move forwards again to move rings
    profileController->generatePath({{0_in, 0_in, 0_deg}, {14_in, 0_in, 0_deg}}, "C");
    profileController->generatePath({{0_in, 0_in, 0_deg}, {5_in, 0_in, 0_deg}}, "D");
    profileController->generatePath({{0_in, 0_in, 0_deg}, {18_in, 0_in, 0_deg}}, "Y");
    profileController->setTarget("C", true);
    liftcontroller->setTarget(2750);
    liftcontroller->waitUntilSettled();
    pros::delay(500);
    liftcontroller->setTarget(1000);
    profileController->waitUntilSettled();
    profileController->setTarget("D");
    profileController->waitUntilSettled();

    //make 90 turn and move to intake rings and 90 again and to move to yellow goal
    PIDTurnAbs(-p);
    profileController->generatePath({{0_in, 0_in, 0_deg}, {24_in, 0_in, 0_deg}}, "Z");
    profileController->setTarget("Z");
    intakecontroller->setTarget(-20000);
    profileController->waitUntilSettled();

    PIDTurnAbs(-(3*p)/2);
    profileController->setTarget("Y");
    tipcontroller->setTarget(3350);
    liftcontroller->setTarget(50);
    profileController->waitUntilSettled();
    jawcontroller->setTarget(-240);
    jawcontroller->waitUntilSettled();
    liftcontroller->setTarget(1000);
    liftcontroller->waitUntilSettled();
    profileControllers->removePath("Z");
    profileController->removePath("Y");

    profileController->generatePath({{0_in, 0_in, 0_deg}, {15_in, -20_in, 0_deg}}, "Y");
    profileController->setTarget("Y");
    liftcontroller->setTarget(2600);
    profileController->waitUntilSettled();

    jawcontroller->setTarget(-20);
    jawcontroller->waitUntilSettled();
    profileController->setTarget("C", true);
    profileController->waitUntilSettled();
    PIDTurnAbs(-6.4);
    profileController->generatePath({{0_in, 0_in, 0_deg}, {28_in, 0_in, 0_deg}}, "Z");
    profileController->setTarget("Z", true);
    tipcontroller->setTarget(3200);
    profileController->waitUntilSettled();
    tipcontroller->setTarget(2200);
    tipcontroller->waitUntilSettled();

    liftcontroller->setTarget(100);
    moveArc(1.9, 55, 20, true, false, 127);
    jawcontroller->setTarget(-250);
    jawcontroller->waitUntilSettled();
    profileController->setTarget("Z");
    liftcontroller->setTarget(2800);
    profileController->waitUntilSettled();
    jawcontroller->setTarget(-20);
    profileController->setTarget("Z", true);
    profileController->waitUntilSettled();
}

//slot 1
void rightAuto(){
    moveHoldFork = false;
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {35_in, 0_in, 0_deg}}, "Z");
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {30_in, 0_in, 0_deg}}, "Y");
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {5_in, 0_in, 0_deg}}, "X");
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, -5_in, 0_deg}}, "A");
//    rprofileController->setTarget("Z");
    setDrive(125, 127);
    jawcontroller->setTarget(-1100);
    pros::delay(790);
    setDrive(0, 0);
    jawcontroller->setTarget(-700);
    rprofileController->waitUntilSettled();
    jawcontroller->waitUntilSettled();

    if (DLF.get_power() > 3){
        while(DLF.get_power() > 3){
            setDrive(-127, -127);
        }
        setDrive(0, 0);
        if (globalPos[0] > 15){
            while (globalPos[0] > 15){
                setDrive(-90, -90);
            }
        }else if (globalPos[0] < 15){
            while(globalPos[0] < 15){
                setDrive(90, 90);
            }
        }
        rprofileController->setTarget("X", true);
        jawcontroller->setTarget(-20);
        rprofileController->waitUntilSettled();
    } else{
        rprofileController->setTarget("Y", true);
        pros::delay(1350);
        jawcontroller->setTarget(-20);
        rprofileController->waitUntilSettled();
    }

//    tipcontroller->setTarget(3200);
//    tipcontroller->waitUntilSettled();
//
//    jawcontroller->setTarget(-1100);
//    jawcontroller->waitUntilSettled();
//    PIDTurnAbs(0.9);
//    setDrive(127, 127);
//    pros::delay(800);
//    jawcontroller->setTarget(-750);
//    jawcontroller->waitUntilSettled();
//
//    if(DLF.get_power() > 3){
//        while(DLF.get_power() > 3){
//            setDrive(-127, -127);
//        }
//    }else{
//        setDrive(-127, -127);
//        pros::delay(800);
//    }
//
//    if (DLF.get_power() > 3){
//        while(DLF.get_power() > 3){
//            setDrive(-127, -127);
//        }
//        setDrive(0, 0);
//
//        rprofileController->setTarget("X", true);
//        jawcontroller->setTarget(-20);
//        rprofileController->waitUntilSettled();
//    } else{
//        rprofileController->setTarget("Y", true);
//        pros::delay(1350);
//        jawcontroller->setTarget(-20);
//        rprofileController->waitUntilSettled();
//    }

//    setDrive(-115, 115);
//    pros::delay(320);
//    setDrive(0, 0);

//    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {15_in, 0_in, 0_deg}}, "B");
//    rprofileController->setTarget("B", true);
//    rprofileController->waitUntilSettled();
//
//    tipcontroller->setTarget(2200);
//    tipcontroller->waitUntilSettled();
//    rprofileController->setTarget("B");
//    intakecontroller->setTarget(-10000);
//    rprofileController->waitUntilSettled();
//
//    tipcontroller->setTarget(3300);
//    tipcontroller->waitUntilSettled();
//    moveHoldFork = true;
}

//slot 2
void rightMod(){
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {15_in, 0_in, 0_deg}}, "Z");
    moveHoldFork = false;
    tipcontroller->setTarget(3300);
    moveArc(1.32, 70, 20, true, false, 127);
    jawcontroller->setTarget(-260);
    jawcontroller->waitUntilSettled();
    tipcontroller->waitUntilSettled();
    moveArc(2.4, 30, 18, true, true, 127);
    tipcontroller->setTarget(2200);
    tipcontroller->waitUntilSettled();

    rprofileController->setTarget("Z");
    intakecontroller->setTarget(-10000);
    rprofileController->waitUntilSettled();
    intakecontroller->waitUntilSettled();
    tipcontroller->setTarget(3300);
    tipcontroller->waitUntilSettled();
    moveHoldFork = true;
}


//slot 3
void leftAuto(){
    moveHoldFork = false;
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {35_in, 0_in, 0_deg}}, "Z");
    setDrive(127, 127);
    pros::delay(1250);
    setDrive(0, 0);
    jawcontroller->setTarget(-250);
    jawcontroller->waitUntilSettled();

    tipcontroller->setTarget(3100);
    if (DLF.get_power() > 3){
        while(globalPos[0] > 17){
            setDrive(-127, -127);
        }
        setDrive(0, 0);
    } else{
        rprofileController->setTarget("Z", true);
        rprofileController->waitUntilSettled();
    }
    setDrive(-110, 110);
    pros::delay(385);
    setDrive(0, 0);

    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {16_in, 0_in, 0_deg}}, "A");
    rprofileController->setTarget("A", true);
    rprofileController->waitUntilSettled();

    tipcontroller->setTarget(2200);
    tipcontroller->waitUntilSettled();
    rprofileController->setTarget("A");
    intakecontroller->setTarget(-10000);
    rprofileController->waitUntilSettled();

    tipcontroller->setTarget(3300);
    tipcontroller->waitUntilSettled();
    moveHoldFork = true;
}

void points(){
    moveHoldFork = false;
    tipcontroller->setTarget(3100);
    tipcontroller->waitUntilSettled();
    rprofileController->generatePath({{0_in, 0_in, 0_deg}, {18_in, 0_in, 0_deg}}, "Z");
    rprofileController->setTarget("Z", true);
    rprofileController->waitUntilSettled();
    tipcontroller->setTarget(2200);
    tipcontroller->waitUntilSettled();
    intakecontroller->setTarget(-10000);
    intakecontroller->waitUntilSettled();
    moveHoldFork = true;
}

int l, r = 0;
const double angE = 20;
void new_right(){
    l = 0;
    r = 0;
//    jawcontroller->setTarget(-1000);
    moveHoldFork = false;
//    tipcontroller->setTarget(-1400);
    while(globalPos[1] < 33) {
        if(globalPos[2] > 0){
            r = 127 - (globalPos[2]*angE);
            l = 127;
        }else if (globalPos[2] < 0){
            l = 127 - (globalPos[2]*angE);
            r = 127;
        }else{
            r = 127;
            l = 127;
        }
        setDrive(l, r);
        if (claw.get_position() > -1000) {
            claw.move_voltage(-9600);
        }else{
            claw.move_voltage(0);
        }
        if (Fork.get_position() > -1400){
            moveHoldFork = false;
            Fork.move_voltage(-12000);
        }else{
            Fork.move_voltage(0);
            moveHoldFork = true;
        }
        pros::delay(20);
    }
    moveHoldFork = true;
    setDrive(0, 0);
    while(claw.get_position() < -500){
        claw.move_voltage(12000);
    }
    pros::delay(200);
    double distance = 15;
    double drop = 26;
    while(globalPos[1] > distance){
        claw.move_voltage(5000);
        if (DLF.get_power() > 5){
            distance = 5;
            setDrive(-127, -127);
            drop = 10;
        }else{
            setDrive(-100, -85);
        }

        if (claw.get_position() > -1000 && globalPos[1] < drop){
            claw.move_voltage(-9600);
        }else{
            claw.move_voltage(0);
        }
    }
    setDrive(0,0);
    pros::delay(250);

    double ang = atan2(-26-globalPos[0], 24-globalPos[1]);
    ang = p + ang;
    pros::lcd::set_text(0, std::to_string(ang));
    PIDTurnAbs(ang);
    while(globalPos[0] > -20){
        setDrive(-50, -50);
    }
    moveHoldFork = false;
//    tipcontroller->setTarget(500);
    liftcontroller->setTarget(1700);
//    tipcontroller->waitUntilSettled();
    while(Fork.get_position() < 1400){
        Fork.move_voltage(12000);
        lift.move_voltage(5000);
    }
    Fork.move_voltage(0);
    lift.move_voltage(0);
    moveHoldFork = true;

    while(globalPos[0] < -19.5){
        setDrive(65, 45);
    }
    moveHoldFork = true;

    PIDTurnAbs(-0.1);
    moveHoldFork = true;

    while(globalPos[1] < 50){
        if(globalPos[2] > 0){
            r = 25 - (globalPos[2]*angE);
            l = 25;
        }else if (globalPos[2] < 0){
            l = 25 - (globalPos[2]*angE);
            r = 25;
        }else{
            r = 25;
            l = 25;
        }
        moveHoldFork = true;
        setDrive(l, r);
        intake.move_voltage(-9600);
    }

    setDrive(-127, -127);
    pros::delay(400);

    setDrive(0, 0);

    moveHoldFork = false;
    while(Fork.get_position() > -1500){
        Fork.move_voltage(-12000);
    }
    moveHoldFork = true;
}

void new_left(){
//    jawcontroller->setTarget(-1000);
    moveHoldFork = false;
//    tipcontroller->setTarget(-1400);
    while(globalPos[1] < 37){
        if(globalPos[2] > 0){
            r = 127 - (globalPos[2]*angE);
            l = 127;
        }else if (globalPos[2] < 0){
            l = 127 - (globalPos[2]*angE);
            r = 127;
        }else{
            r = 127;
            l = 127;
        }
        setDrive(l, r);
        if (claw.get_position() > -1100) {
            claw.move_voltage(-9600);
        }else{
            claw.move_voltage(0);
        }
        if(Fork.get_position() > -1000){
            moveHoldFork = false;
            Fork.move_voltage(-12000);
        }
        pros::delay(20);
    }
    moveHoldFork = true;
    setDrive(0, 0);
//    jawcontroller->setTarget(-500);
//    jawcontroller->waitUntilSettled();
    while(claw.get_position() < -550){
        claw.move_voltage(12000);
    }
    pros::delay(200);
    while(globalPos[1] > 14){
        claw.move_voltage(5000);
        if (DLF.get_power() > 5){
            setDrive(-127, -127);
        }else{
            setDrive(-105, -105);
        }
        if (claw.get_position() > -1000 && globalPos[1] < 20){
            claw.move_voltage(-9600);
        }else{
            claw.move_voltage(0);
        }
    }
    setDrive(0,0);
    pros::delay(250);
    PIDTurnAbs(1.64);
//    double ang = atan2(-3-globalPos[0], 0-globalPos[1]);
//    ang = p + ang;
//    pros::lcd::set_text(0, std::to_string(ang));
//    PIDTurnAbs(ang);
    while(globalPos[0] > -15){
        setDrive(-60, -60);
    }
//    pros::delay(700);
    moveHoldFork = false;
    while(Fork.get_position() < 1300){
        Fork.move_voltage(12000);
        lift.move_voltage(8000);
    }
    Fork.move_voltage(0);
    lift.move_voltage(0);
    moveHoldFork = true;


    setDrive(40, 40);
    pros::delay(900);
    for(int i = 0; i < 2; i++){
        while(globalPos[0] < 7){
            setDrive(25, 25);
            intake.move_voltage(-9600);
        }
        if (i != 1){
            setDrive(-60, -60);
            pros::delay(800);
        }
    }
    setDrive(0, 0);
    moveHoldFork = false;
    while(Fork.get_position() > -2000){
        Fork.move_voltage(-12000);
    }
    moveHoldFork = true;
}
