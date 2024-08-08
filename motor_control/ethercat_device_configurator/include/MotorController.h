#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <vector>
#include <string>
#include <optional>
#include "Motor.h"

class MotorController
{
private:
    std::vector<Motor> motors;

    std::string getMotorNames() const;

public:
    MotorController(const std::vector<Motor> &motors);

    std::vector<Motor> getMotors() const;
    std::vector<Motor> getJoints() const;
    Motor getGripper() const;

    void setMotorTargetPosition(int index, float position);
    std::optional<float> getMotorTargetPosition(int index) const;
    void setMotorReadPosition(int index, float position);
    std::optional<float> getMotorReadPosition(int index) const;

    void setMotorTargetVelocity(int index, float velocity);
    std::optional<float> getMotorTargetVelocity(int index) const;
    void setMotorReadVelocity(int index, float velocity);
    std::optional<float> getMotorReadVelocity(int index) const;

    void setMotorTargetTorque(int index, float torque);
    std::optional<float> getMotorTargetTorque(int index) const;
    void setMotorReadTorque(int index, float torque);
    std::optional<float> getMotorReadTorque(int index) const;

    void setGripperPosition(float position);
    std::optional<float> getGripperPosition() const;

    Motor getMotor(const std::string &name) const;

    friend std::ostream &operator<<(std::ostream &os, const MotorController &controller);

    void initializeMotors();
};

#endif // MOTORCONTROLLER_H
