#include "gtest/gtest.h"
#include "MotorController.h"

TEST(MotorControllerTest, GetMotorByName)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    Motor retrieved_motor1 = motorController.getMotor("Motor1");
    EXPECT_EQ(retrieved_motor1.getName(), "Motor1");

    Motor retrieved_gripper = motorController.getMotor("Gripper");
    EXPECT_EQ(retrieved_gripper.getName(), "Gripper");
}

TEST(MotorControllerTest, GetMotorByInvalidName)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    EXPECT_THROW(motorController.getMotor("InvalidMotor"), std::invalid_argument);
}

TEST(MotorControllerTest, SetMotorTargetPosition)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetPosition(0, 90.0f);
    EXPECT_EQ(motorController.getMotorTargetPosition(0).value(), 90.0f);
}

TEST(MotorControllerTest, SetInvalidMotorTargetPosition)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetPosition(0, 200.0f);                   // Out of range
    EXPECT_FALSE(motorController.getMotorTargetPosition(0).has_value()); // Target position should not be set
}

TEST(MotorControllerTest, SetMotorReadPosition)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorReadPosition(0, 90.0f);
    EXPECT_EQ(motorController.getMotorReadPosition(0).value(), 90.0f);
}

TEST(MotorControllerTest, SetMotorTargetVelocity)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetVelocity(0, 50.0f);
    EXPECT_EQ(motorController.getMotorTargetVelocity(0).value(), 50.0f);
}

TEST(MotorControllerTest, SetInvalidMotorTargetVelocity)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetVelocity(0, 200.0f);                   // Out of range
    EXPECT_FALSE(motorController.getMotorTargetVelocity(0).has_value()); // Target velocity should not be set
}

TEST(MotorControllerTest, SetMotorReadVelocity)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorReadVelocity(0, 50.0f);
    EXPECT_EQ(motorController.getMotorReadVelocity(0).value(), 50.0f);
}

TEST(MotorControllerTest, SetMotorTargetTorque)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetTorque(0, 30.0f);
    EXPECT_EQ(motorController.getMotorTargetTorque(0).value(), 30.0f);
}

TEST(MotorControllerTest, SetInvalidMotorTargetTorque)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorTargetTorque(0, 60.0f);                    // Out of range
    EXPECT_FALSE(motorController.getMotorTargetTorque(0).has_value()); // Target torque should not be set
}

TEST(MotorControllerTest, SetMotorReadTorque)
{
    Motor motor1("Motor1", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor motor2("Motor2", 100.0f, 50.0f, 0.0f, 180.0f, 1);
    Motor gripper("Gripper", 50.0f, 30.0f, 0.0f, 90.0f, 1);

    std::vector<Motor> motors = {motor1, motor2, gripper};
    MotorController motorController(motors);

    motorController.setMotorReadTorque(0, 30.0f);
    EXPECT_EQ(motorController.getMotorReadTorque(0).value(), 30.0f);
}
