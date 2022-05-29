#include "main.h"

void skills();

void rightAuto();

void rightMod();

void leftAuto();

void points();

void new_right();

void new_left();


extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<AsyncMotionProfileController> profileControllers;
extern std::shared_ptr<AsyncMotionProfileController> rprofileController;
extern std::shared_ptr<AsyncMotionProfileController> profileControllerm;
extern std::shared_ptr<AsyncMotionProfileController> profileController;
extern std::shared_ptr<AsyncPositionController<double, double>> jawcontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> liftcontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> tipcontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> intakecontroller;