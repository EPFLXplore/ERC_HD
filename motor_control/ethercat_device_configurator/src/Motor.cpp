#include "Motor.h"

Motor::Motor(const std::string &name, float max_velocity, float max_torque,
             float min_position, float max_position, int direction)
    : name(name), max_velocity(max_velocity), max_torque(max_torque),
      min_position(min_position), max_position(max_position), direction(direction),
      target_position(std::nullopt), read_position(std::nullopt),
      target_velocity(std::nullopt), read_velocity(std::nullopt),
      target_torque(std::nullopt), read_torque(std::nullopt) {}

std::string Motor::getName() const { return name; }
float Motor::getMaxVelocity() const { return max_velocity; }
float Motor::getMaxTorque() const { return max_torque; }
float Motor::getMinPosition() const { return min_position; }
float Motor::getMaxPosition() const { return max_position; }
int Motor::getDirection() const { return direction; }

std::optional<float> Motor::getTargetPosition() const { return target_position; }
void Motor::setTargetPosition(float position)
{
    if (position < min_position || position > max_position)
    {
        spdlog::warn("Target position {} is out of range [{} - {}]", position, min_position, max_position);
    }
    else
    {
        this->target_position = position;
    }
}

std::optional<float> Motor::getReadPosition() const { return read_position; }
void Motor::setReadPosition(float position)
{
    this->read_position = position;
}

std::optional<float> Motor::getTargetVelocity() const { return target_velocity; }
void Motor::setTargetVelocity(float velocity)
{
    if (velocity < 0 || velocity > max_velocity)
    {
        spdlog::warn("Target velocity {} is out of range [0 - {}]", velocity, max_velocity);
    }
    else
    {
        this->target_velocity = velocity;
    }
}

std::optional<float> Motor::getReadVelocity() const { return read_velocity; }
void Motor::setReadVelocity(float velocity)
{
    this->read_velocity = velocity;
}

std::optional<float> Motor::getTargetTorque() const { return target_torque; }
void Motor::setTargetTorque(float torque)
{
    if (torque < 0 || torque > max_torque)
    {
        spdlog::warn("Target torque {} is out of range [0 - {}]", torque, max_torque);
    }
    else
    {
        this->target_torque = torque;
    }
}

std::optional<float> Motor::getReadTorque() const { return read_torque; }
void Motor::setReadTorque(float torque)
{
    this->read_torque = torque;
}

std::ostream &operator<<(std::ostream &os, const Motor &motor)
{
    os << "Motor[name: " << motor.name
       << ", max_velocity: " << motor.max_velocity
       << ", max_torque: " << motor.max_torque
       << ", min_position: " << motor.min_position
       << ", max_position: " << motor.max_position
       << ", direction: " << motor.direction;

    if (motor.target_position.has_value())
    {
        os << ", target_position: " << motor.target_position.value();
    }
    else
    {
        os << ", target_position: not set";
    }

    if (motor.read_position.has_value())
    {
        os << ", read_position: " << motor.read_position.value();
    }
    else
    {
        os << ", read_position: not set";
    }

    if (motor.target_velocity.has_value())
    {
        os << ", target_velocity: " << motor.target_velocity.value();
    }
    else
    {
        os << ", target_velocity: not set";
    }

    if (motor.read_velocity.has_value())
    {
        os << ", read_velocity: " << motor.read_velocity.value();
    }
    else
    {
        os << ", read_velocity: not set";
    }

    if (motor.target_torque.has_value())
    {
        os << ", target_torque: " << motor.target_torque.value();
    }
    else
    {
        os << ", target_torque: not set";
    }

    if (motor.read_torque.has_value())
    {
        os << ", read_torque: " << motor.read_torque.value();
    }
    else
    {
        os << ", read_torque: not set";
    }

    os << "]";
    return os;
}
