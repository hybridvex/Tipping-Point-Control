#include "main.h"
#include <initializer_list>
#include <utility>

using namespace pros;

#define apply(f, ...) for (auto* motor : motors) {motor->f(__VA_ARGS__);}

/// A group of motors for easy control
template<int N>
class MotorGroup {
private:
	/// Stores the N pointers to motors
	Motor* motors[N];
public:
	/// Construct a MotorGroup from the listed motors (of size N)
	MotorGroup(std::initializer_list<Motor*> data): motors{} {
		std::copy(data.begin(), data.end(), std::begin(motors));
	}
	
	// Copies of motor functions that are used

	void move_velocity(int32_t velocity) {apply(move_velocity, velocity);}
	void set_brake_mode(motor_brake_mode_e_t mode) {apply(set_brake_mode, mode);}
};

Controller ctrl {E_CONTROLLER_MASTER};
Motor r1 {1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES};
Motor r2 {7, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES};
Motor r3 {3, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES};
MotorGroup<3> right {&r1, &r2, &r3};
Motor l1 {4, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES};
Motor l2 {5, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES};
Motor l3 {6, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES};
MotorGroup<3> left {&l1, &l2, &l3};

Motor hlift {11, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES};
Motor intake {20, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES};

IMU inert {10};

ADIDigitalOut claw {'A'};
ADIDigitalOut blift {'B'};
ADIDigitalIn claw_switch {'C'};
ADIDigitalIn back_switch {'D'};

vision_signature_s_t YELLOW_MOGO = Vision::signature_from_utility(1, 1681, 2071, 1876, -4131, -3429, -3780, 5.700, 0);
vision_signature_s_t BLUE_MOGO = Vision::signature_from_utility(2, -3419, -2703, -3061, 4121, 11077, 7599, 3.000, 0);
vision_signature_s_t RED_MOGO = Vision::signature_from_utility(3, 7673, 8267, 7970, -449, -201, -325, 7.100, 0);
Vision vs {1};

/// Transform an analog joystick value into an output power (for wheels)
int32_t process_analog(double v) {
	if (abs(v) < 20) return 0;
	return v/127.0*600.0;
}

/// Initialize before anything else runs
void initialize() {
	// Calibrate
	inert.reset();
	/// We want coast if not running auton
	left.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right.set_brake_mode(E_MOTOR_BRAKE_COAST);
	hlift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// Vision sensor
	vs.set_exposure(52);
	vs.set_wifi_mode(0);
}


void disabled() {}


void competition_initialize() {}

/// Run a PID loop with the given constants, to goal. Will stop when error is within thresh.
/// `progress()` gives the current progress to determine the error;
/// `output(double power)` should power motors necessary to achieve the goal
void pid(
	double kP, double kI, double kD, double goal, double thresh,
	std::function<double()> progress, std::function<void(double)> output
) {
	double p, error, integral, derivative, perror;
	do {
		p = progress();
		perror = error;
		error = goal - p;
		integral += error;
		derivative = error - perror;
		output(kP * error + kI * integral + kD * derivative);
		pros::delay(5);
	} while (error > thresh);
}

constexpr double METRES_TO_TICKS = 1.0;
/// Move the distance in metres - rough, so use vision or limit switch when close
void move(int distance) {
	pid(1.0, 1.0, 1.0, distance*METRES_TO_TICKS, 0.1, [&](){
		return l1.get_position();
	}, [&](double power){
		left.move_velocity(power);
		right.move_velocity(power);
	});
}

/// Turn the specified angle, should be accurate due to inertial
void turn(int deg) {
	inert.tare();
	pid(1.0, 1.0, 1.0, deg, 0.1, [&](){
		return inert.get_yaw();
	}, [&](double power){
		left.move_velocity(power);
		right.move_velocity(-power);
	});
}

/// Balance the mogo on the ramp in front of us
void balance_mogo() {

}

/// Chase forwards until a mogo is reached (with the limit switch)
void chase_mogo() {
	// TODO: hook up vision sensor to stay on target
	left.move_velocity(600);
	right.move_velocity(600);
	while (claw_switch.get_new_press() == 0) {
		pros::delay(10);
	}
	claw.set_value(true);
	left.move_velocity(0);
	right.move_velocity(0);
}

/// Drive the robot up the ramp, using the inertial to detect progress
void self_ramp() {
	left.move_velocity(300);
	right.move_velocity(300);
	while (abs(inert.get_pitch()) > 10) {pros::delay(5);}
	left.move_velocity(-100);
	right.move_velocity(-100);
	pros::delay(300);
	left.move_velocity(0);
	right.move_velocity(0);
}

/// Runs the autonomous routine
// TODO: better system than downloading twice for skills vs match
void autonomous() {
	left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	self_ramp();
	left.set_brake_mode(E_MOTOR_BRAKE_COAST);
	right.set_brake_mode(E_MOTOR_BRAKE_COAST);
}

/// Transforms two digital buttons to a motor output power
int digilog(bool up, bool down, int scale) {
	return (up ? scale : 0) + (down ? -scale : 0);
}

enum Intake {
	Forwards, Backwards, None
};
Intake intake_state = Intake::None;

/// Driver control.
void opcontrol() {
	while (true) {
		// Wheels
		left.move_velocity(process_analog((double) ctrl.get_analog(ANALOG_LEFT_Y)));
		right.move_velocity(process_analog((double) ctrl.get_analog(ANALOG_RIGHT_Y)));
		// Pneumatic systems
		if (ctrl.get_digital_new_press(DIGITAL_UP)) blift.set_value(true);
		if (ctrl.get_digital_new_press(DIGITAL_DOWN)) blift.set_value(false);
		if (ctrl.get_digital_new_press(DIGITAL_L1)) claw.set_value(false);
		if (ctrl.get_digital_new_press(DIGITAL_L2)) claw.set_value(true);
		// Lift
		hlift.move_velocity(digilog(ctrl.get_digital(DIGITAL_R1),ctrl.get_digital(DIGITAL_R2),100));
		// Intake
		// refactor please
		if (ctrl.get_digital_new_press(DIGITAL_X)) {
			intake_state = intake_state == Intake::Forwards ? Intake::None : Intake::Forwards;
		}
		if (ctrl.get_digital_new_press(DIGITAL_B)) {
			intake_state = intake_state == Intake::Backwards ? Intake::None : Intake::Backwards;
		}
		switch (intake_state) {
			case Intake::Forwards:
				intake.move_velocity(600);
				break;
			case Intake::Backwards:
				intake.move_velocity(-600);
				break;
			case Intake::None:
				intake.move_velocity(0);
				break;
		}
		// Show largest objects to controller screen
		// ctrl.clear();
		// vision_object_s_t obj_arr [3];
		// auto objs = vs.read_by_size(0, 3, obj_arr);
		// for (int i = 0; i < objs; i++) {
		// 	auto obj = obj_arr[i];
		// 	ctrl.print(i, 1, "Mid: (%d, %d), Dim: (%d, %d)", obj.x_middle_coord, obj.y_middle_coord, obj.width, obj.height);
		// }

		// Haptic feedback
		if (claw_switch.get_new_press()) ctrl.rumble("-");
		if (back_switch.get_new_press()) ctrl.rumble("-.");

		// Auton for testing
		// TODO: remove before a comp
		// if (ctrl.get_digital_new_press(DIGITAL_A)) autonomous();
		pros::delay(20);
	}
}
