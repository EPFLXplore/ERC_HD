#include <gtest/gtest.h>
#include "MotorConfigLoader.h"
#include "Motor.h"

class MotorConfigLoaderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Setup code if necessary
    }

    void TearDown() override
    {
        // Cleanup code if necessary
    }

    // Helper method to create a temporary YAML file
    std::string createTempYamlFile(const std::string &content)
    {
        std::ofstream file("temp_test_config.yaml");
        file << content;
        file.close();
        return "temp_test_config.yaml";
    }
};

TEST_F(MotorConfigLoaderTest, LoadDeviceConfigsTest)
{
    std::string yamlContent = R"(
used_joints:
  - Motor1
  - Motor2
Motor1:
  max_velocity: 100.0
  max_torque: 50.0
  min_position: 0.0
  max_position: "PI*2"
  direction: 1
Motor2:
  max_velocity: 150.0
  max_torque: 60.0
  min_position: -10.0
  max_position: "PI"
  direction: -1
)";
    std::string filePath = createTempYamlFile(yamlContent);
    MotorConfigLoader loader(filePath);
    std::vector<Motor> motors = loader.loadDeviceConfigs();

    ASSERT_EQ(motors.size(), 2);

    EXPECT_EQ(motors[0].getName(), "Motor1");
    EXPECT_FLOAT_EQ(motors[0].getMaxVelocity(), 100.0);
    EXPECT_FLOAT_EQ(motors[0].getMaxTorque(), 50.0);
    EXPECT_FLOAT_EQ(motors[0].getMinPosition(), 0.0);
    EXPECT_FLOAT_EQ(motors[0].getMaxPosition(), M_PI * 2);
    EXPECT_EQ(motors[0].getDirection(), 1);

    EXPECT_EQ(motors[1].getName(), "Motor2");
    EXPECT_FLOAT_EQ(motors[1].getMaxVelocity(), 150.0);
    EXPECT_FLOAT_EQ(motors[1].getMaxTorque(), 60.0);
    EXPECT_FLOAT_EQ(motors[1].getMinPosition(), -10.0);
    EXPECT_FLOAT_EQ(motors[1].getMaxPosition(), M_PI);
    EXPECT_EQ(motors[1].getDirection(), -1);
}
