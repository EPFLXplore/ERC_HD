#include "MotorController.h"
#include <stdexcept>
#include <iostream>

MotorController::MotorController(const std::vector<Motor> &motors)
    : motors(motors) {}

std::vector<Motor> MotorController::getMotors() const
{
  return motors;
}

std::vector<Motor> MotorController::getJoints() const
{
  std::vector<Motor> joints;
  for (const auto &motor : motors)
  {
    if (motor.getName() != "gripper")
    {
      joints.push_back(motor);
    }
  }
  return joints;
}

Motor MotorController::getGripper() const
{
  return getMotor("gripper");
}

void MotorController::setMotorTargetPosition(int index, float position)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setTargetPosition(position);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorTargetPosition(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getTargetPosition();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setMotorReadPosition(int index, float position)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setReadPosition(position);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorReadPosition(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getReadPosition();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setMotorTargetVelocity(int index, float velocity)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setTargetVelocity(velocity);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorTargetVelocity(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getTargetVelocity();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setMotorReadVelocity(int index, float velocity)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setReadVelocity(velocity);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorReadVelocity(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getReadVelocity();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setMotorTargetTorque(int index, float torque)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setTargetTorque(torque);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorTargetTorque(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getTargetTorque();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setMotorReadTorque(int index, float torque)
{
  if (index >= 0 && index < motors.size())
  {
    motors[index].setReadTorque(torque);
  }
  else
  {
    throw std::out_of_range("Invalid motor index");
  }
}

std::optional<float> MotorController::getMotorReadTorque(int index) const
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index].getReadTorque();
  }
  else
  {
    return std::nullopt;
  }
}

void MotorController::setGripperPosition(float position)
{
  getMotor("gripper").setTargetPosition(position);
}

std::optional<float> MotorController::getGripperPosition() const
{
  return getMotor("gripper").getTargetPosition();
}

Motor MotorController::getMotor(const std::string &name) const
{
  for (const auto &motor : motors)
  {
    if (motor.getName() == name)
    {
      return motor;
    }
  }
  throw std::invalid_argument("Motor with the name '" + name + "' not found. Available motors: " + getMotorNames());
}

std::string MotorController::getMotorNames() const
{
  std::string motorNames;
  for (const auto &motor : motors)
  {
    motorNames += motor.getName() + ", ";
  }
  return motorNames;
}

std::ostream &operator<<(std::ostream &os, const MotorController &controller)
{
  os << "MotorController with " << controller.motors.size() << " motors:\n";
  for (const auto &motor : controller.motors)
  {
    os << motor << "\n";
  }
  return os;
}

void MotorController::initializeMotors()
{
  for (auto &motor : motors)
  {
    motor.setTargetPosition(0.0);
    motor.setReadPosition(0.0);
    motor.setTargetVelocity(0.0);
    motor.setReadVelocity(0.0);
    motor.setTargetTorque(0.0);
    motor.setReadTorque(0.0);
  }
}
