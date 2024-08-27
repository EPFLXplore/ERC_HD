#ifndef MOTOR_H
#define MOTOR_H

#include <string>
#include <iostream>
#include <stdexcept>
#include <atomic>
#include <mutex>
#include <spdlog/spdlog.h>
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include "custom_msg/msg/hd/MotorCommands.msg"


class Motor
{
private:
    const std::string name;
    const float max_velocity;
    const float max_torque;
    const float min_position;
    const float max_position;
    const int direction;

    std::atomic<int> mode;
    std::atomic<double> target_position;
    std::atomic<double> read_position;
    std::atomic<double> target_velocity;
    std::atomic<double> read_velocity;
    std::atomic<double> target_torque;
    std::atomic<double> read_torque;
    std::atomic<double> read_current;

public:
    Motor(const std::string &name, float max_velocity, float max_torque,
          float min_position, float max_position, int direction);

    // Delete copy constructor and copy assignment operator
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    // Allow move constructor and move assignment operator
    Motor(Motor&&) noexcept = default;
    Motor& operator=(Motor&&) noexcept = default;

    std::string getName() const;
    float getMaxVelocity() const;
    float getMaxTorque() const;
    float getMinPosition() const;
    float getMaxPosition() const;
    int getDirection() const;
    int getMode() const;
    double getReadCurrent() const;
    

    double getTargetPosition() const;
    void setTargetPosition(double position);
    double getReadPosition() const;
    void setReadPosition(double position);

    double getTargetVelocity() const;
    void setTargetVelocity(double velocity);
    double getReadVelocity() const;
    void setReadVelocity(double velocity);

    double getTargetTorque() const;
    void setTargetTorque(double torque);
    double getReadTorque() const;
    void setReadTorque(double torque);
    void setMode(int new_mode);
    void setReadCurrent(double current);

    maxon::Command getCommand() const;

    friend std::ostream &operator<<(std::ostream &os, const Motor &motor);
};

#endif // MOTOR_H
