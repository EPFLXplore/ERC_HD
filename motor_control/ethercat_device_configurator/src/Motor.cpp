#include "Motor.h"

// Constructor
Motor::Motor(const std::string &name, float max_velocity, float max_torque,
             float min_position, float max_position, int direction)
    : name(name), max_velocity(max_velocity), max_torque(max_torque),
      min_position(min_position), max_position(max_position), direction(direction),
      target_position(0.0), read_position(0.0),
      target_velocity(0.0), read_velocity(0.0),
      target_torque(0.0), read_torque(0.0) {}

// Getters
std::string Motor::getName() const { return name; }
float Motor::getMaxVelocity() const { return max_velocity; }
float Motor::getMaxTorque() const { return max_torque; }
float Motor::getMinPosition() const { return min_position; }
float Motor::getMaxPosition() const { return max_position; }
int Motor::getDirection() const { return direction; }

// Thread-safe getter and setter for target_position
double Motor::getTargetPosition() const {
    return target_position.load();
}

void Motor::setTargetPosition(double position) {
    if (position < min_position || position > max_position) {
        spdlog::warn("Target position {} is out of range [{} - {}]", position, min_position, max_position);
    } else {
        target_position.store(position);
    }
}

// Thread-safe getter and setter for read_position
double Motor::getReadPosition() const {
    return read_position.load();
}

void Motor::setReadPosition(double position) {
    read_position.store(position);
}

// Thread-safe getter and setter for target_velocity
double Motor::getTargetVelocity() const {
    return target_velocity.load();
}

void Motor::setTargetVelocity(double velocity) {
    if (velocity < -max_velocity || velocity > max_velocity) {
       spdlog::warn("Target velocity {} is out of range [-{} - {}]", velocity, max_velocity, max_velocity);
    } else {
        target_velocity.store(velocity);
    }
}

// Thread-safe getter and setter for read_velocity
double Motor::getReadVelocity() const {
    return read_velocity.load();
}

void Motor::setReadVelocity(double velocity) {
    read_velocity.store(velocity);
}

// Thread-safe getter and setter for target_torque
double Motor::getTargetTorque() const {
    return target_torque.load();
}

void Motor::setTargetTorque(double torque) {
    if (torque < -max_torque || torque > max_torque) {
        spdlog::warn("Target torque {} is out of range [-{} - {}]", torque, max_torque, max_torque);
    } else {
        target_torque.store(torque);
    }
}

// Thread-safe getter and setter for read_torque
double Motor::getReadTorque() const {
    return read_torque.load();
}

void Motor::setReadTorque(double torque) {
    read_torque.store(torque);
}

void Motor::setMode(int new_mode) {
    mode.store(new_mode);
}

int Motor::getMode() const {
    return mode.load();
}

// Generate command for the motor
maxon::Command Motor::getCommand() const {
    // std::cout << *this << std::endl;
    maxon::Command command;
    command.setTargetPosition(getTargetPosition());
    command.setTargetVelocity(getTargetVelocity());
    command.setTargetTorque( getTargetTorque());
    if( getMode() == hd_interfaces::msg::MotorCommands::POSITION_MODE)
    {
        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
    }
    else if( getMode() == hd_interfaces::msg::MotorCommands::VELOCITY_MODE)
    {
        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
    }
    else if( getMode() == hd_interfaces::msg::MotorCommands::TORQUE_MODE)
    {
        command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
    }
    // std::cout << command << std::endl;
    return command;
}

// Overloading the << operator for Motor
std::ostream &operator<<(std::ostream &os, const Motor &motor) {
    os << std::fixed << std::setprecision(6);
    os << "Motor[name: " << motor.name
        << ", max_velocity: " << std::setw(3) << std::setfill('0') << motor.max_velocity
        << ", max_torque: " << std::setw(3) << std::setfill('0') << motor.max_torque
        << ", min_position: " << std::setw(3) << std::setfill('0') << motor.min_position
        << ", max_position: " << std::setw(3) << std::setfill('0') << motor.max_position
        << ", direction: " << motor.direction
        << ", target_position: " << std::setw(9) << std::setfill('0') << motor.getTargetPosition()
        << ", read_position: " << std::setw(9) << std::setfill('0') << motor.getReadPosition()
        << ", target_velocity: " << std::setw(9) << std::setfill('0') << motor.getTargetVelocity()
        << ", read_velocity: " << std::setw(9) << std::setfill('0') << motor.getReadVelocity()
        << ", target_torque: " << std::setw(9) << std::setfill('0') << motor.getTargetTorque()
        << ", read_torque: " << std::setw(9) << std::setfill('0') << motor.getReadTorque()
        << ", mode: " << motor.getMode()
        << ", read_current: " << std::setw(9) << std::setfill('0') << motor.getReadCurrent()
        << "]";
    return os;
}

double Motor::getReadCurrent() const {
    return read_current.load();
}

void Motor::setReadCurrent(double current) {
    read_current.store(current);
}