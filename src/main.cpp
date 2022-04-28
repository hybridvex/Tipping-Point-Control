#include "main.h"
#include "pros/vision.h"
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
	void move_voltage(int32_t voltage) {apply(move_voltage, voltage);}
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
ADIDigitalOut mogo_block {'E'};

constexpr int32_t YELLOW_SIG = 1;
constexpr int32_t BLUE_SIG = 2;
constexpr int32_t RED_SIG = 3;

vision_signature_s_t YELLOW_MOGO = Vision::signature_from_utility(YELLOW_SIG, 95, 1519, 806, -3341, -2601, -2972, 2.500, 0);
vision_signature_s_t BLUE_MOGO = Vision::signature_from_utility(BLUE_SIG, -3091, -2277, -2684, 9599, 14073, 11836, 3.000, 0);
vision_signature_s_t RED_MOGO = Vision::signature_from_utility(RED_SIG, 4801, 6877, 5840, -65, 567, 250, 2.500, 0);
Vision vs {12};

/// Transform an analog joystick value into an output power (for wheels)
int32_t process_analog(double v) {
	if (abs(v) < 20) return 0;
	return v/127.0*600.0;
	// return powf(v/127.0, 2.0)*600.0
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
	vs.set_signature(YELLOW_SIG, &YELLOW_MOGO);
	vs.set_signature(BLUE_SIG, &BLUE_MOGO);
	vs.set_signature(RED_SIG, &RED_MOGO);
	// stay in limits
	claw.set_value(true);
}


void disabled() {}


void competition_initialize() {
	
}

constexpr int VS_MID = 128;

class PID {
private:
	const double kP, kI, kD, thresh;
	double error = 0, derivative = 0, integral = 0;
	int doneFor = 0;
	bool in_target() {
		return error > 0 ? error < thresh : -error < thresh;
	}
public:
	double target = 0;
	PID(double kP, double kI, double kD, double thresh): kP{kP}, kI{kI}, kD{kD}, thresh{thresh} {}
	double update(double sensor) {
		double new_error = target - sensor;
		derivative = new_error - error;
		error = new_error;
		integral += error;
		if (in_target()) doneFor++; else doneFor = 0;
		printf("PID is %f %f %f\n", error, derivative, integral);
		return kP * error + kI * integral + kD * derivative;
	}
	void reset() {
		error, derivative, integral, target = 0, 0, 0, 0;
		doneFor = 0;
	}
	bool done() {
		return in_target() && doneFor >= 15;
	}
};

PID drivePID {20.0, 0.02, 100.0, 20.0};
// +ve left, -ve right I think
PID turnPID {380.0, 0.01, 170.0, 3.0};

void chase_mogo() {
	drivePID.reset();
	turnPID.reset();
	drivePID.target = 3000.0;
	turnPID.target = VS_MID;
	l1.tare_position();
	while (true) {
		if (inert.is_calibrating()) ctrl.rumble("-");
		double drive = drivePID.update(l1.get_position());
		auto mogo = vs.get_by_sig(0, YELLOW_SIG);
		if (mogo.signature == 255) return;
		double turn = turnPID.update(mogo.x_middle_coord);
		left.move_voltage(drive + turn);
		right.move_voltage(drive - turn);
		delay(10);
	}
}

void turn(double deg) {
	turnPID.reset();
	turnPID.target = deg;
	inert.tare_rotation();
	while (!turnPID.done()) {
		double turn = turnPID.update(inert.get_rotation());
		left.move_voltage(turn);
		right.move_voltage(-turn);
		delay(10);
	}
}

void drive(double distance) {
	turnPID.reset();
	turnPID.target = 0;
	inert.tare_rotation();
	drivePID.reset();
	drivePID.target = distance;
	l1.tare_position();
	while (!drivePID.done()) {
		double turn = turnPID.update(inert.get_rotation());
		double drive = drivePID.update(l1.get_position());
		left.move_voltage(drive + 0.1 * turn);
		right.move_voltage(drive - 0.1 * turn);
		delay(10);
	}
}

void autonomous() {
	left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	claw.set_value(false);
	drive(2000.0);
	
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

bool mogo_block_active = true;

/// Driver control.
void opcontrol() {
	while (true) {
		// Wheels
		left.move_velocity(process_analog((double) ctrl.get_analog(ANALOG_LEFT_Y)));
		right.move_velocity(process_analog((double) ctrl.get_analog(ANALOG_RIGHT_Y)));
		// Pneumatic systems
		if (ctrl.get_digital_new_press(DIGITAL_UP)) blift.set_value(false);
		if (ctrl.get_digital_new_press(DIGITAL_DOWN)) blift.set_value(true);
		if (ctrl.get_digital_new_press(DIGITAL_L1)) claw.set_value(false);
		if (ctrl.get_digital_new_press(DIGITAL_L2)) claw.set_value(true);
		if (ctrl.get_digital_new_press(DIGITAL_Y)) mogo_block.set_value(mogo_block_active ^= true);
		
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
		if (ctrl.get_digital_new_press(DIGITAL_A)) autonomous();
		pros::delay(20);
	}
}
