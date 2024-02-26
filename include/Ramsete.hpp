#pragma once

#include <memory>
#include "motion_profiling.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "Eigen/Dense"
#include "pros/motor_group.hpp"

class Ramsete {
public:
    Ramsete(double kp, double kd, std::shared_ptr<ProfileGenerator> generator,
            std::shared_ptr<voss::localizer::AbstractLocalizer> l, const double DRIVE_CHASSIS_WIDTH,
            std::initializer_list<int8_t> leftMotors,
            std::initializer_list<int8_t> rightMotors);

    void follow(virtualPath &target_path, double max);

    std::pair<double, double> update(int i);

    void execute();

    voss::Point absToLocal(voss::Pose currentPose, voss::Point pt);


private:
    void updateGain(double desired_v, double desired_w);

    bool arrived = false;
    double d;
    double gain, kp, kd;
    const double DRIVE_CHASSIS_WIDTH;
    Pose desirePose;
    std::shared_ptr<ProfileGenerator> generator;
    std::vector<ProfilePoint> profile;
    std::shared_ptr<voss::localizer::AbstractLocalizer> l;
    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;
};

