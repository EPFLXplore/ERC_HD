#ifndef MOTOR_H
#define MOTOR_H

#include <string>
#include <optional>
#include <iostream>
#include <stdexcept>
#include <spdlog/spdlog.h>

class Motor
{
private:
    const std::string name;
    const float max_velocity;
    const float max_torque;
    const float min_position;
    const float max_position;
    const int direction;

    std::optional<float> target_position;
    std::optional<float> read_position;
    std::optional<float> target_velocity;
    std::optional<float> read_velocity;
    std::optional<float> target_torque;
    std::optional<float> read_torque;

public:
    Motor(const std::string &name, float max_velocity, float max_torque,
          float min_position, float max_position, int direction);

    std::string getName() const;
    float getMaxVelocity() const;
    float getMaxTorque() const;
    float getMinPosition() const;
    float getMaxPosition() const;
    int getDirection() const;

    std::optional<float> getTargetPosition() const;
    void setTargetPosition(float position);
    std::optional<float> getReadPosition() const;
    void setReadPosition(float position);

    std::optional<float> getTargetVelocity() const;
    void setTargetVelocity(float velocity);
    std::optional<float> getReadVelocity() const;
    void setReadVelocity(float velocity);

    std::optional<float> getTargetTorque() const;
    void setTargetTorque(float torque);
    std::optional<float> getReadTorque() const;
    void setReadTorque(float torque);

    friend std::ostream &operator<<(std::ostream &os, const Motor &motor);
};

#endif // MOTOR_H