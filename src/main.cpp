#include "main.h"
#include "Ramsete.hpp"
#include "VOSS/api.hpp"

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
 *
 */

auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
        .with_left_motors({-2, -3, -6, -5})
        .with_right_motors({11, 12, 19, 20})
        .with_left_right_tpi(18.24) // 19.5
        .with_track_width(9.75)     // 3.558
        .with_imu(13)
        .build();


auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
        .with_linear_constants(7, 0.02, 40)
        .with_angular_constants(170, 0, 700)
        .with_exit_error(1.0)
        .with_angular_exit_error(1.0)
        .with_min_error(5)
        .with_settle_time(200)
        .build();

voss::chassis::DiffChassis chassis({-2, -3, -6, -5}, {11, 12, 19, 20}, pid,
                                   8);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
    odom->begin_localization();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    double maxSpeed = 450.0 / 60 * 2.75 * M_PI;

    const double delta_d = 0.1;
    const int sample_points = 0;
    const int benchmark_samples = 100;
    const double TRACK_WIDTH = 15;

    // Test Motion Profile
    auto constraints = new Constraints(maxSpeed, maxSpeed * 3, 0.1, maxSpeed * 3, maxSpeed * 100, TRACK_WIDTH);

    auto profileGenerator = std::make_shared<ProfileGenerator>(constraints, delta_d);
    // benchmark profile gen

    Ramsete ramsete(1, 1, profileGenerator, odom, TRACK_WIDTH, {-2, -3, -6, -5}, {11, 12, 19, 20});


    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (1) {
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            odom->set_pose({0, 0, 0});
            auto testPath = CubicBezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}, sample_points);
            ramsete.follow(testPath, 100.0);
        }

        pros::delay(10);
    }
}