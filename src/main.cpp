#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
 // some like config stuff that will be useful trust
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-3, -4}, pros::MotorGearset::red);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({1, 2}, pros::MotorGearset::red);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 13.5, lemlib::Omniwheel::NEW_325, 100, 2);
pros::Imu imu(17);
pros::Rotation rotation_sensor(18);
lemlib::TrackingWheel tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_2 , 2.72);
lemlib::OdomSensors sensors(&tracking_wheel,nullptr,nullptr,nullptr, &imu);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

pros::Distance left_distance_sensor();
pros::Distance top_distance_sensor();
pros::Distance right_distance_sensor();
pros::Distance bottom_distance_sensor();
// these 4 variables are the distance between the distance sensors and the rotational center of the robot, which is where we set pose
double left_sensor_distance_error = 13.5/2 + 0.875;
double top_sensor_distance_error = 14.5/2 + 0.875;
double right_sensor_distance_error = 13.5/2 + 0.875;
double bottom_sensor_distance_error = 14.5/2 + 0.875;
// the pneuneuneumatics
pros::adi::Pneumatics claw_piston('G', false); // piston that makes claw open and close, claw is closed when piston is extended
int claw_piston_state = 1; // 1 is closed, 0 is open
pros::adi::Pneumatics intake_piston('H', false); // piston for the intake
int intake_piston_state = 0; // 1 is extended, 0 is retracted
// pros::adi::Pneumatics claw_piston2('A', false); // piston that makes the claw go up and down
// int claw_piston2_state = 0; // 1 is extended, 0 is retracted
// the motors for cascade and intake
pros::Motor cascade_motor(5); // motor for cascade
pros::Motor cascade_motor2(6); // motor for cascade
pros::Motor intake_motor(7); // motor for intake
/**
 * A callback function for LLEMU's right and left button.
 * It is for changing the autonomous section
 */
int autonomous_selector = 0;

void on_right_button() {
	autonomous_selector++;
	if (autonomous_selector > 3) {
		autonomous_selector = 0;
	}
}

void on_left_button(){
    autonomous_selector--;
	if (autonomous_selector < 0) {
		autonomous_selector = 3;
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::lcd::register_btn0_cb(on_left_button); // register left button callback
    pros::lcd::register_btn2_cb(on_right_button); // register right button callback
    cascade_motor2.set_reversed(true); // reverse cascade motor 2
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::delay(500);
        }
    });
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

void autonomous() {
    if(autonomous_selector == 0){
        // run autonomous when team goal is left
        claw_piston.extend(); // claw up + close
        claw_piston_state = 1;
        pros::delay(300);
        chassis.arcade(50,0); // go forward
        pros::delay(500);
        chassis.arcade(0,0); // stop
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300); 
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        intake_piston.extend(); // intake open
        intake_piston_state = 1;
        chassis.arcade(-50,0); // go back
        pros::delay(700);
        chassis.arcade(0,0); // stop
        // spin intake roller
        intake_motor.move(70); // intake roller spin
        pros::delay(800);
        intake_motor.move(0); // intake roller stop
        intake_piston.retract(); // intake close
        intake_piston_state = 0;
        pros::delay(300);
        claw_piston.extend(); // claw close
        claw_piston_state = 1;
        pros::delay(300);
        cascade_motor.move_absolute(200, 100); // cascade up
        cascade_motor2.move_absolute(200, 100); // cascade up
        pros::delay(700);
        chassis.arcade(50,0); // go forward
        pros::delay(800);
        chassis.arcade(0,0); // stop
        // cascade up claw open (again)
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300);
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        chassis.setPose(-23.55,-55.15,0);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        cascade_motor.move_absolute(0, 100); // cascade down
        cascade_motor2.move_absolute(0, 100); // cascade down
        pros::delay(700);
        chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
        chassis.waitUntilDone();
        chassis.turnToHeading(0,1000); // turn to face front
        chassis.moveToPoint(0,-60,1000); // move forward
        chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
    } 
    else if(autonomous_selector == 1){
        // run autonomous when team goal is right
        claw_piston.extend(); // claw up + close
        claw_piston_state = 1;
        pros::delay(300);
        chassis.arcade(50,0); // go forward
        pros::delay(500);
        chassis.arcade(0,0); // stop
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300); 
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        intake_piston.extend(); // intake open
        intake_piston_state = 1;
        chassis.arcade(-50,0); // go back
        pros::delay(700);
        chassis.arcade(0,0); // stop
        // spin intake roller
        intake_motor.move(70); // intake roller spin
        pros::delay(800);
        intake_motor.move(0); // intake roller stop
        intake_piston.retract(); // intake close
        intake_piston_state = 0;
        pros::delay(300);
        claw_piston.extend(); // claw close
        claw_piston_state = 1;
        pros::delay(300);
        cascade_motor.move_absolute(200, 100); // cascade up
        cascade_motor2.move_absolute(200, 100); // cascade up
        pros::delay(700);
        chassis.arcade(50,0); // go forward
        pros::delay(800);
        chassis.arcade(0,0); // stop
        // cascade up claw open (again)
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300);
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        chassis.setPose(23.55,-55.15,0);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        cascade_motor.move_absolute(0, 100); // cascade down
        cascade_motor2.move_absolute(0, 100); // cascade down
        pros::delay(700);
        chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
        chassis.waitUntilDone();
        chassis.turnToHeading(0,1000); // turn to face front
        chassis.moveToPoint(0,-60,1000); // move forward
        chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
    }
    else if(autonomous_selector == 2){
        // run autonomous skills code
        claw_piston.extend(); // claw up + close
        claw_piston_state = 1;
        pros::delay(300);
        chassis.arcade(50,0); // go forward
        pros::delay(500);
        chassis.arcade(0,0); // stop
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300); 
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        intake_piston.extend(); // intake open
        intake_piston_state = 1;
        chassis.arcade(-50,0); // go back
        pros::delay(700);
        chassis.arcade(0,0); // stop
        // spin intake roller
        intake_motor.move(70); // intake roller spin
        pros::delay(800);
        intake_motor.move(0); // intake roller stop
        intake_piston.retract(); // intake close
        intake_piston_state = 0;
        pros::delay(300);
        claw_piston.extend(); // claw close
        claw_piston_state = 1;
        pros::delay(300);
        cascade_motor.move_absolute(200, 100); // cascade up
        cascade_motor2.move_absolute(200, 100); // cascade up
        pros::delay(700);
        chassis.arcade(50,0); // go forward
        pros::delay(800);
        chassis.arcade(0,0); // stop
        // cascade up claw open (again)
        // claw_piston2.extend(); // claw up
        // claw_piston2_state = 1;
        pros::delay(300);
        claw_piston.retract(); // claw open
        claw_piston_state = 0;
        pros::delay(300);
        chassis.setPose(-23.55,-55.15,0);
        // claw_piston2.retract(); // claw down
        // claw_piston2_state = 0;
        pros::delay(300);
        cascade_motor.move_absolute(0, 100); // cascade down
        cascade_motor2.move_absolute(0, 100); // cascade down
        pros::delay(700);
        chassis.moveToPoint(0,-65,1500); // move to toggle, activating it once
        chassis.waitUntilDone();
        chassis.turnToHeading(0,1000); // turn to face front
        chassis.moveToPoint(0,-60,1000); // move forward
        chassis.moveToPoint(0,-68,1000); // move back, toggling toggle
    }
    else{
        // run manual skills code
        // uh why would you do anything in here its manual are we serious rn
    }
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


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// double arcade control scheme
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            cascade_motor.move(60); // cascade up
            cascade_motor2.move(60); // cascade up
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            cascade_motor.move(-60); // cascade down
            cascade_motor2.move(-60); // cascade down
        }
        else{
            cascade_motor.move(0); // cascade stop
            cascade_motor2.move(0); // cascade stop
        }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            if(claw_piston_state == 1){
                claw_piston.retract(); // claw open
                claw_piston_state = 0;
            }
            else{
                claw_piston.extend(); // claw close
                claw_piston_state = 1;
            }
        }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake_motor.move(127); // intake roller spin
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake_motor.move(-127); // intake roller spin reverse aka outtake
        }
        else{
            intake_motor.move(0); // intake roller stop
        }
        // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        //     if(claw_piston2_state == 1){
        //         claw_piston2.retract(); // claw down
        //         claw_piston2_state = 0;
        //     }
        //     else{
        //         claw_piston2.extend(); // claw up
        //         claw_piston2_state = 1;
        //     }
        // }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            if(intake_piston_state == 1){
                intake_piston.retract(); // intake close
                intake_piston_state = 0;
            }
            else{
                intake_piston.extend(); // intake open
                intake_piston_state = 1;
            }
        }
        // delay to save resources
        pros::delay(15);                         // Run for 5 ms then update
	}
}

/*
 * This is the tracking algorithm that tracks the robot
 * its basically odomentry
 * This should be used in conjunction with every autonomous and driver control place
 * basically speaking, you need multiple threads (yay.)
*/
// void tracking(){
//     //i don't think we need this
// }
