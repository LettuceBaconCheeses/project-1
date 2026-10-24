#pragma once
#include "api.h"
#include <cmath>

class drivetrainController {
public:
    pros::Motor leftDrivetrain;
    pros::Motor rightDrivetrain;

    pros::Rotation forwardTracking;
    pros::Rotation sidewaysTracking;

    pros::IMU inertialReader;

    drivetrainController(int leftPort, int rightPort, int forwardTrackingPort, int sidewaysTrackingPort, int imuPort, bool leftReversed)
        : leftDrivetrain(leftReversed ? -leftPort : leftPort),
          rightDrivetrain(leftReversed ? rightPort : -rightPort),

          forwardTracking(forwardTrackingPort),
          sidewaysTracking(sidewaysTrackingPort),
          inertialReader(imuPort) {}

    /************************* INITIALIZE *************************/
    void initInertial() {
        inertialReader.reset(true);  
    }

    void initLCD() {
        pros::lcd::initialize();
    }

    /************************* FUNCTIONS *************************/
    void PDController(float targetRotation, float velocity, float kp, float kd) {
    float rotation, error, corr;

    while (true) {
        rotation = inertialReader.get_rotation();

        error = targetRotation - rotation;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        if (fabs(error) < 1) error = 0;

        float pValue = error * kp;

        corr = pValue;

        // Calculate the velocities and run motors
        float leftVelo = clamp(velocity + corr, -100, 100);
        float rightVelo = clamp(velocity - corr, -100, 100);
        leftDrivetrain.move_velocity(leftVelo);
        rightDrivetrain.move_velocity(rightVelo);

        pros::delay(20);
    }

    leftDrivetrain.move_velocity(0);
    rightDrivetrain.move_velocity(0);
    }

    /************************* DEBUGGING *************************/
    void runTelemetry() {
        while (true) {
            pros::lcd::print(0, "Rotation: %.3f", inertialReader.get_rotation());
            pros::lcd::print(1, "Forward: %.2f", forwardTracking.get_position() / 100.0);
            pros::lcd::print(2, "Sideways: %.2f", sidewaysTracking.get_position() / 100.0);
            pros::delay(100);
        }
    }

private:
/************************* HELPER FUNCTIONS *************************/
    static float clamp(float value, float minValue, float maxValue) {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }
};