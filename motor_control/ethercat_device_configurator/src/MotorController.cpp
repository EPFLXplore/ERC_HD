#include "MotorController.h"
#include <stdexcept>
#include <iostream>

// Constructor that accepts a vector of unique pointers to motors
MotorController::MotorController(std::vector<std::unique_ptr<Motor>> motors)
    : motors(std::move(motors)) {}

// Constructor that loads motors from a config file
MotorController::MotorController(const std::string &config_path)
{
    MotorConfigLoader motor_loader(config_path);
    this->motors = motor_loader.loadDeviceConfigs();
}

// Return a vector of references to motors
std::vector<std::reference_wrapper<Motor>> MotorController::getMotors() const
{
    std::vector<std::reference_wrapper<Motor>> motor_refs;
    for (const auto& motor : motors)
    {
        motor_refs.push_back(*motor);
    }
    return motor_refs;
}

// Return a vector of references to motors except the gripper
std::vector<std::reference_wrapper<Motor>> MotorController::getJoints() const
{
    std::vector<std::reference_wrapper<Motor>> joints;
    for (const auto& motor : motors)
    {
        if (motor->getName() != "Gripper")
        {
            joints.push_back(*motor);
        }
    }
    return joints;
}

// Return a reference to the gripper motor
Motor& MotorController::getGripper()
{
    return getMotor("Gripper");
}

// Set the target position of a motor by index
void MotorController::setMotorTargetPosition(size_t index, float position)
{
    if (index < motors.size())
    {
        motors[index]->setTargetPosition(position);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the target position of a motor by index
float MotorController::getMotorTargetPosition(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getTargetPosition();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the read position of a motor by index
void MotorController::setMotorReadPosition(size_t index, float position)
{
    if (index < motors.size())
    {
        motors[index]->setReadPosition(position);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the read position of a motor by index
float MotorController::getMotorReadPosition(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getReadPosition();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the target velocity of a motor by index
void MotorController::setMotorTargetVelocity(size_t index, float velocity)
{
    if (index < motors.size())
    {
        motors[index]->setTargetVelocity(velocity);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the target velocity of a motor by index
float MotorController::getMotorTargetVelocity(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getTargetVelocity();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the read velocity of a motor by index
void MotorController::setMotorReadVelocity(size_t index, float velocity)
{
    if (index < motors.size())
    {
        motors[index]->setReadVelocity(velocity);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the read velocity of a motor by index
float MotorController::getMotorReadVelocity(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getReadVelocity();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the target torque of a motor by index
void MotorController::setMotorTargetTorque(size_t index, float torque)
{
    if (index < motors.size())
    {
        motors[index]->setTargetTorque(torque);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the target torque of a motor by index
float MotorController::getMotorTargetTorque(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getTargetTorque();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the read torque of a motor by index
void MotorController::setMotorReadTorque(size_t index, float torque)
{
    if (index < motors.size())
    {
        motors[index]->setReadTorque(torque);
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Get the read torque of a motor by index
float MotorController::getMotorReadTorque(size_t index) const
{
    if (index < motors.size())
    {
        return motors[index]->getReadTorque();
    }
    else
    {
        throw std::out_of_range("Invalid motor index");
    }
}

// Set the position of the gripper motor
void MotorController::setGripperPosition(float position)
{
    getMotor("Gripper").setTargetPosition(position);
}

// Get the position of the gripper motor
float MotorController::getGripperPosition() 
{
    return getMotor("Gripper").getTargetPosition();
}

// Get a reference to a motor by name
Motor& MotorController::getMotor(const std::string &name)
{
    for (const auto& motor : motors)
    {
        if (motor->getName() == name)
        {
            return *motor;
        }
    }
    throw std::invalid_argument("Motor with the name '" + name + "' not found. Available motors: " + getMotorNames());
}

// Get the names of all motors as a comma-separated string
std::string MotorController::getMotorNames() const
{
    std::string motorNames;
    for (const auto& motor : motors)
    {
        motorNames += motor->getName() + ", ";
    }
    return motorNames;
}

// Overload the << operator for MotorController
std::ostream &operator<<(std::ostream &os, const MotorController &controller)
{
    os << "MotorController with " << controller.motors.size() << " motors:\n";
    for (const auto& motor : controller.motors)
    {
        os << *motor << "\n";
    }
    return os;
}

// Initialize all motors to default values
void MotorController::initializeMotors()
{
    for (auto& motor : motors)
    {
        motor->setTargetPosition(0.0);
        motor->setReadPosition(0.0);
        motor->setTargetVelocity(0.0);
        motor->setReadVelocity(0.0);
        motor->setTargetTorque(0.0);
        motor->setReadTorque(0.0);
        motor->setMode(hd_interfaces::msg::MotorCommands::POSITION_MODE);
    }
}


// Check if the controller contains a motor with the given name
bool MotorController::containsMotor(const std::string &name) const
{
    for (const auto& motor : motors)
    {
        if (motor->getName() == name)
        {
            return true;
        }
    }
    return false;
}
