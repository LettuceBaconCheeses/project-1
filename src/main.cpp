#include "main.h"
#include "api.h"
#include "drivetrain.hpp"

using namespace pros;

// DEFINE PORTS

drivetrainController chassis (1, 2, 3, 4, 5, false);

void initialize() {
	chassis.initLCD();
	chassis.initInertial();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	Task telemetry([] { chassis.runTelemetry(); });
}