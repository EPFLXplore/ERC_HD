#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <functional>
#include "Motor.h"
#include "MotorConfigLoader.h"
#include "hd_interfaces/msg/motor_commands.hpp"


// using namespace std;

class MotorController
{
private:
    std::vector<std::unique_ptr<Motor>> motors;


public:
    MotorController(std::vector<std::unique_ptr<Motor>> motors);
    MotorController(const std::string &config_path);

    std::vector<std::reference_wrapper<Motor>> getMotors() const;
    std::vector<std::reference_wrapper<Motor>> getJoints() const;
    Motor& getGripper();

    bool containsMotor(const std::string &name) const;  // New method declaration
    std::string getMotorNames() const;


    void setMotorTargetPosition(size_t index, float position);
    float getMotorTargetPosition(size_t index) const;
    void setMotorReadPosition(size_t index, float position);
    float getMotorReadPosition(size_t index) const;

    void setMotorTargetVelocity(size_t index, float velocity);
    float getMotorTargetVelocity(size_t index) const;
    void setMotorReadVelocity(size_t index, float velocity);
    float getMotorReadVelocity(size_t index) const;

    void setMotorTargetTorque(size_t index, float torque);
    float getMotorTargetTorque(size_t index) const;
    void setMotorReadTorque(size_t index, float torque);
    float getMotorReadTorque(size_t index) const;

    void setGripperPosition(float position);
    float getGripperPosition();

    Motor& getMotor(const std::string &name);

    friend std::ostream &operator<<(std::ostream &os, const MotorController &controller);

    void initializeMotors();
};

#endif // MOTORCONTROLLER_H
