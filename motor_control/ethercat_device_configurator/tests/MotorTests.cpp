#include "Motor.h"
#include "gtest/gtest.h"

TEST(MotorTest, SetValidTargetPosition)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetPosition(90.0f);
    EXPECT_EQ(motor.getTargetPosition().value(), 90.0f);
}

TEST(MotorTest, SetInvalidTargetPosition)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetPosition(200.0f);                     // Out of range
    EXPECT_FALSE(motor.getTargetPosition().has_value()); // Target position should not be set
}

TEST(MotorTest, SetValidReadPosition)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setReadPosition(90.0f);
    EXPECT_EQ(motor.getReadPosition().value(), 90.0f);
}

TEST(MotorTest, SetValidTargetVelocity)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetVelocity(50.0f);
    EXPECT_EQ(motor.getTargetVelocity().value(), 50.0f);
}

TEST(MotorTest, SetInvalidTargetVelocity)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetVelocity(200.0f);                     // Out of range
    EXPECT_FALSE(motor.getTargetVelocity().has_value()); // Target velocity should not be set
}

TEST(MotorTest, SetValidReadVelocity)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setReadVelocity(50.0f);
    EXPECT_EQ(motor.getReadVelocity().value(), 50.0f);
}

TEST(MotorTest, SetValidTargetTorque)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetTorque(30.0f);
    EXPECT_EQ(motor.getTargetTorque().value(), 30.0f);
}

TEST(MotorTest, SetInvalidTargetTorque)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setTargetTorque(60.0f);                      // Out of range
    EXPECT_FALSE(motor.getTargetTorque().has_value()); // Target torque should not be set
}

TEST(MotorTest, SetValidReadTorque)
{
    Motor motor("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    motor.setReadTorque(30.0f);
    EXPECT_EQ(motor.getReadTorque().value(), 30.0f);
}
