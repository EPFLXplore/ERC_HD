#include "MotorConfigLoader.h"

MotorConfigLoader::MotorConfigLoader(const std::string &filePath) : filePath(filePath)
{
    parser = ExpressionParser();
}

std::vector<std::unique_ptr<Motor>> MotorConfigLoader::loadDeviceConfigs()
{
    std::vector<std::unique_ptr<Motor>> motors;
    std::vector<std::string> deviceNames;

    try
    {
        YAML::Node config = YAML::LoadFile(filePath);

        for (const auto &value : config["used_joints"])
        {
            deviceNames.push_back(value.as<std::string>());
        }

        for (const auto &deviceName : deviceNames)
        {
            if (config[deviceName])
            {
                std::string name = deviceName;
                float max_velocity = config[deviceName]["max_velocity"].as<float>();
                float max_torque = config[deviceName]["max_torque"].as<float>();

                std::string min_position_str = config[deviceName]["min_position"].as<std::string>();
                std::string max_position_str = config[deviceName]["max_position"].as<std::string>();
                parser.replacePI(min_position_str);
                parser.replacePI(max_position_str);

                float min_position = parser.string2double(min_position_str);
                float max_position = parser.string2double(max_position_str);
                int direction = config[deviceName]["direction"].as<int>();
                
                motors.emplace_back(std::make_unique<Motor>(name, max_velocity, max_torque, min_position, max_position, direction));
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
    }
    return motors;
}
