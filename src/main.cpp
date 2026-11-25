#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include <cmath>
#include <new>
// some like config stuff that will be useful trust
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup
    left_mg({-8, -7},
            pros::MotorGearset::blue); // Left: port 8 reversed, port 7 normal
pros::MotorGroup right_mg(
    {9, 10},
    pros::MotorGearset::blue); // Right: port 9 normal, port 10 reversed
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.4,
                              lemlib::Omniwheel::NEW_325, 100, 2);
pros::Imu imu(11);
pros::Rotation
    vertical_rotation_sensor(-5); // Port 5, reversed for forward travel.
lemlib::TrackingWheel vertical_wheel(&vertical_rotation_sensor,
                                     lemlib::Omniwheel::NEW_2, -0.5);
constexpr int horizontal_sensor_port = 6;
// TODO: Set the measured signed offset from the center of rotation, in inches.
constexpr double horizontal_wheel_offset = 0.0;
pros::Rotation horizontal_rotation_sensor(horizontal_sensor_port);
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation_sensor,
                                       lemlib::Omniwheel::NEW_2,
                                       horizontal_wheel_offset);
lemlib::OdomSensors sensors(&vertical_wheel, nullptr, &horizontal_wheel,
                            nullptr, &imu);
// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10, // proportional gain (kP)
                       0,  // integral gain (kI)
                       25, // derivative gain (kD)
                       0,  // anti windup
                       0,  // smfall error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angular_controller(2,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       10, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in degrees
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in degrees
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

pros::Distance left_distance_sensor();
pros::Distance top_distance_sensor();
pros::Distance right_distance_sensor();
pros::Distance bottom_distance_sensor();
// these 4 variables are the distance between the distance sensors and the
// rotational center of the robot, which is where we set pose
double left_sensor_distance_error = 13.5 / 2 + 0.875;
double top_sensor_distance_error = 14.5 / 2 + 0.875;
double right_sensor_distance_error = 13.5 / 2 + 0.875;
double bottom_sensor_distance_error = 14.5 / 2 + 0.875;
// the motors for cascade and intake
pros::Motor cascade_motor(15);  // motor for cascade
pros::Motor cascade_motor2(12); // motor for cascade
pros::Motor intake_motor(2);    // motor for intake
/**
 * A callback function for LLEMU's right and left button.
 * It is for changing the autonomous section
 */
int autonomous_selector = 0;
int selected_gain = 0; // 0: kP, 1: kI, 2: kD

lemlib::ControllerSettings &tuning_settings(int test) {
  return test < 2 ? lateral_controller : angular_controller;
}

void show_tuning_values() {
  const auto &settings = tuning_settings(autonomous_selector);
  const char *const gain_names[] = {"kP", "kI", "kD"};
  pros::lcd::print(1, "%s PID: P %.2f I %.2f D %.2f",
                   autonomous_selector < 2 ? "Drive" : "Turn", settings.kP,
                   settings.kI, settings.kD);
  pros::lcd::print(2, "B:%s Y/A:+/- %.2f", gain_names[selected_gain],
                   selected_gain == 1 ? 0.01 : 0.1);
}

void show_tuning_selection() {
  const char *const names[] = {"Drive +10 in", "Drive -10 in", "Turn +90 deg",
                               "Turn -90 deg"};
  pros::lcd::print(0, "PID test: %s", names[autonomous_selector]);
  show_tuning_values();
}

void on_right_button() {
  autonomous_selector++;
  if (autonomous_selector > 3) {
    autonomous_selector = 0;
  }
  show_tuning_selection();
}

void on_left_button() {
  autonomous_selector--;
  if (autonomous_selector < 0) {
    autonomous_selector = 3;
  }
  show_tuning_selection();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  pros::lcd::initialize();                     // initialize brain screen
  chassis.calibrate();                         // calibrate sensors
  pros::lcd::register_btn0_cb(on_left_button); // register left button callback
  pros::lcd::register_btn2_cb(
      on_right_button); // register right button callback
  show_tuning_selection();
  cascade_motor2.set_reversed(true); // reverse cascade motor 2
  cascade_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cascade_motor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_motor.set_gearing(pros::MotorGearset::blue);    // set intake
  cascade_motor.set_gearing(pros::MotorGearset::green);  // set cascade
  cascade_motor2.set_gearing(pros::MotorGearset::green); // set cascade
  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
  // pros::Task screenTask([&]() {
  //     while (true) {
  //         // print robot location to the brain screen
  //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
  //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
  //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); //
  //         heading pros::delay(500);
  //     }
  // });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() { show_tuning_selection(); }

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

void autonomous() {
  // Reset the robot to the same physical starting position for each run.
  constexpr int tuning_timeout_ms = 5000;
  const int selected_test = autonomous_selector;
  if (selected_test < 2 && lateral_controller.kP == 0 &&
      lateral_controller.kI == 0 && lateral_controller.kD == 0) {
    pros::lcd::set_text(0, "Drive PID gains are zero");
    pros::lcd::set_text(1, "Set lateral kP to test");
    pros::lcd::clear_line(2);
    return;
  }
  // LemLib stores PID gains inside the chassis. Reconstruct the selected PID
  // only while no motion is running so the next test uses the edited values.
  auto &settings = tuning_settings(selected_test);
  auto &pid = selected_test < 2 ? chassis.lateralPID : chassis.angularPID;
  pid.~PID();
  new (&pid)
      lemlib::PID(settings.kP, settings.kI, settings.kD, settings.windupRange);
  chassis.setPose(0, 0, 0);
  pros::lcd::set_text(0, "PID test running");
  const std::uint32_t start = pros::millis();
  switch (selected_test) {
  case 0:
    chassis.moveToPoint(0, 10, tuning_timeout_ms);
    break;
  case 1:
    chassis.moveToPoint(0, -10, tuning_timeout_ms, {.forwards = false});
    break;
  case 2:
    chassis.turnToHeading(90, tuning_timeout_ms);
    break;
  case 3:
    chassis.turnToHeading(-90, tuning_timeout_ms);
    break;
  }
  // waitUntilDone() synchronizes with the asynchronous motion. A separate
  // display task provides live readings without changing PID during a motion.
  pros::Task display_task([selected_test, start]() {
    const double target = selected_test == 0   ? 10
                          : selected_test == 1 ? -10
                          : selected_test == 2 ? 90
                                               : -90;
    while (true) {
      const lemlib::Pose pose = chassis.getPose();
      double error = target - (selected_test < 2 ? pose.y : pose.theta);
      if (selected_test >= 2)
        error = std::remainder(error, 360.0);
      pros::lcd::print(3, "Target %.1f Err %.2f", target, error);
      pros::lcd::print(4, "X %.2f Y %.2f", pose.x, pose.y);
      pros::lcd::print(5, "Heading %.1f T %.1fs", pose.theta,
                       (pros::millis() - start) / 1000.0);
      pros::delay(100);
    }
  });
  chassis.waitUntilDone();
  display_task.remove();
  const lemlib::Pose result = chassis.getPose();
  pros::lcd::set_text(0, pros::millis() - start >= tuning_timeout_ms
                             ? "PID test timed out"
                             : "PID test finished");
  pros::lcd::print(1, "X: %.2f Y: %.2f", result.x, result.y);
  pros::lcd::print(2, "Heading: %.2f deg", result.theta);
  // if (autonomous_selector == 0) {
  //   // run autonomous when team goal is left
  //   // pros::delay(300);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(500);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.arcade(-50,0); // go back
  //   // pros::delay(700);
  //   // chassis.arcade(0,0); // stop
  //   // // spin intake roller
  //   // intake_motor.move(70); // intake roller spin
  //   // pros::delay(800);
  //   // intake_motor.move(0); // intake roller stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(200, 100); // cascade up
  //   // cascade_motor2.move_absolute(200, 100); // cascade up
  //   // pros::delay(700);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(800);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.setPose(-23.55,-55.15,0);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(0, 100); // cascade down
  //   // cascade_motor2.move_absolute(0, 100); // cascade down
  //   // pros::delay(700);
  //   // chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
  //   // chassis.waitUntilDone();
  //   // chassis.turnToHeading(0,1000); // turn to face front
  //   // chassis.moveToPoint(0,-60,1000); // move forward
  //   // chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
  //   chassis.setPose(0, -75, 0);
  //   chassis.moveToPose(-23.55, -66, 270, 1500);

  // } else if (autonomous_selector == 1) {
  //   // run autonomous when team goal is right
  //   // pros::delay(300);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(500);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.arcade(-50,0); // go back
  //   // pros::delay(700);
  //   // chassis.arcade(0,0); // stop
  //   // // spin intake roller
  //   // intake_motor.move(70); // intake roller spin
  //   // pros::delay(800);
  //   // intake_motor.move(0); // intake roller stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(200, 100); // cascade up
  //   // cascade_motor2.move_absolute(200, 100); // cascade up
  //   // pros::delay(700);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(800);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.setPose(23.55,-55.15,0);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(0, 100); // cascade down
  //   // cascade_motor2.move_absolute(0, 100); // cascade down
  //   // pros::delay(700);
  //   // chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
  //   // chassis.waitUntilDone();
  //   // chassis.turnToHeading(0,1000); // turn to face front
  //   // chassis.moveToPoint(0,-60,1000); // move forward
  //   // chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
  // } else if (autonomous_selector == 2) {
  //   // run autonomous skills code
  //   // pros::delay(300);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(500);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.arcade(-50,0); // go back
  //   // pros::delay(700);
  //   // chassis.arcade(0,0); // stop
  //   // // spin intake roller
  //   // intake_motor.move(70); // intake roller spin
  //   // pros::delay(800);
  //   // intake_motor.move(0); // intake roller stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(200, 100); // cascade up
  //   // cascade_motor2.move_absolute(200, 100); // cascade up
  //   // pros::delay(700);
  //   // chassis.arcade(50,0); // go forward
  //   // pros::delay(800);
  //   // chassis.arcade(0,0); // stop
  //   // pros::delay(300);
  //   // pros::delay(300);
  //   // chassis.setPose(-23.55,-55.15,0);
  //   // pros::delay(300);
  //   // cascade_motor.move_absolute(0, 100); // cascade down
  //   // cascade_motor2.move_absolute(0, 100); // cascade down
  //   // pros::delay(700);
  //   // chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
  //   // chassis.waitUntilDone();
  //   // chassis.turnToHeading(0,1000); // turn to face front
  //   // chassis.moveToPoint(0,-60,1000); // move forward
  //   // chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
  // } else {
  //   // run manual skills code
  //   // uh why would you do anything in here its manual are we serious rn
  // }
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
  // Autonomous can leave asynchronous LemLib motions queued.
  chassis.cancelAllMotions();
  chassis.arcade(0, 0);

  int screen_ticks = 0;
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      selected_gain = (selected_gain + 1) % 3;
      screen_ticks = 0;
    }
    const bool increase =
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
    const bool decrease =
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    const bool increase_large =
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
    const bool decrease_large =
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    if (increase || decrease || increase_large || decrease_large) {
      auto &settings = tuning_settings(autonomous_selector);
      float *gain = selected_gain == 0   ? &settings.kP
                    : selected_gain == 1 ? &settings.kI
                                         : &settings.kD;
      const float step = selected_gain == 1 ? 0.01f : 0.1f;
      const float change = step * ((increase ? 1 : 0) - (decrease ? 1 : 0) +
                                   (increase_large ? 10 : 0) -
                                   (decrease_large ? 10 : 0));
      *gain = std::fmax(0.0f, std::round((*gain + change) / step) * step);
      screen_ticks = 0;
    }
    // X runs the selected test only without a field controller connected.
    if (!pros::competition::is_connected() &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      chassis.arcade(0, 0);
      autonomous();
      pros::delay(
          2000); // Keep the result visible before driver display resumes.
      screen_ticks = 0;
      continue;
    }
    // Split arcade: left stick forward/backward, right stick steering.
    const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(throttle, turn);

    // Hold R1 to raise the cascade; hold L1 to lower it.
    int cascade_power = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      cascade_power = 127;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      cascade_power = -127;
    }
    if (cascade_power == 0) {
      cascade_motor.brake();
      cascade_motor2.brake();
    } else {
      cascade_motor.move(cascade_power);
      cascade_motor2.move(cascade_power);
    }

    // Hold R2 to intake; hold L2 to outtake.
    int intake_power = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake_power = 127;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake_power = -127;
    }
    intake_motor.move(intake_power);

    // Show that driver control is running and receiving joystick values.
    if (screen_ticks == 0) {
      show_tuning_selection();
      pros::lcd::set_text(3, "X: run test  Brain: select");
      pros::lcd::print(4, "Vertical sensor: %.1f deg",
                       vertical_rotation_sensor.get_position() / 100.0);
      pros::lcd::print(5, "Odom Y: %.2f in", chassis.getPose().y);
    }
    screen_ticks = (screen_ticks + 1) % 10;
    pros::delay(20);
  }
}

/*
 * This is the tracking algorithm that tracks the robot
 * its basically odomentry
 * This should be used in conjunction with every autonomous and driver control
 * place basically speaking, you need multiple threads (yay.)
 */
// void tracking(){
//     //i don't think we need this
// }
