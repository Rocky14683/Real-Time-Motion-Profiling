#include "Ramsete.hpp"


Ramsete::Ramsete(double kp, double kd, std::shared_ptr<ProfileGenerator> generator,
                 std::shared_ptr<voss::localizer::AbstractLocalizer> l, const double DRIVE_CHASSIS_WIDTH,
                 std::initializer_list<int8_t> leftMotors,
                 std::initializer_list<int8_t> rightMotors) : generator(
        generator), l(l), DRIVE_CHASSIS_WIDTH(DRIVE_CHASSIS_WIDTH) {
    this->leftMotors = std::make_unique<pros::MotorGroup>(leftMotors);
    this->rightMotors = std::make_unique<pros::MotorGroup>(rightMotors);
    this->kp = kp;
    this->kd = kd;
    this->gain = 0;
    this->d = 0;
}

void Ramsete::updateGain(double desired_v, double desired_w) {
    this->gain = 2.0 * this->kd * sqrt(pow(desired_w, 2) + this->kp * pow(desired_v, 2));
}

void Ramsete::follow(virtualPath &targetPath, double max) {
    this->generator->generateProfile(&targetPath);
    this->profile = this->generator->getProfile();
    for (int i = 0; i < profile.size(); i++) {
        auto [v, w] = update(i);
        double angEffect = (w * DRIVE_CHASSIS_WIDTH / 2.0);
        double left = v + angEffect;
        double right = v - angEffect;
        double v_max = std::max(fabs(left), fabs(right));
        if (v_max > max) {
            left = left * max / v_max;
            right = right * max / v_max;
        }
        this->leftMotors->move_voltage(120 * left);
        this->rightMotors->move_voltage(120 * right);
    }
}

std::pair<double, double> Ramsete::update(int i) {
    double h = this->l->get_orientation_rad();
    auto profilePt = profile.at(i);
    double v_d = profilePt.vel;
    double w_d = profilePt.vel * profilePt.curvature;
    voss::Pose currentPose = {this->l->get_position().x, this->l->get_position().y, h};

    this->updateGain(v_d, w_d);
    Eigen::Matrix3Xd posMatrix = Eigen::Matrix3Xd({
                                                          {cos(h),  sin(h), 0},
                                                          {-sin(h), cos(h), 0},
                                                          {0,       0,      1}
                                                  });

    Eigen::Matrix3Xd deltaMatrix = Eigen::Matrix3Xd({
                                                            {profilePt.x - currentPose.x},
                                                            {profilePt.y - currentPose.y},
                                                            {profilePt.theta - currentPose.theta}
                                                    });

    Eigen::Matrix3Xd errorMatrix = posMatrix * deltaMatrix;
    voss::Pose error = {errorMatrix.col(0).x(), errorMatrix.col(0).y(), errorMatrix.col(0).z()};
    double v = v_d * cos(error.theta) + this->gain * error.x;
    double w = w_d + gain * error.theta + (this->kp * v_d * sin(error.theta) * error.y) / error.theta;

    return {v, w};
}





