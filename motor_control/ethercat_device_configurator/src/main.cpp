#include "spdlog/sinks/stdout_color_sinks.h"
#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <vector>
#include "MotorController.h"
#include "MotorConfigLoader.h"

int main()
{
    // Set up spdlog to log to both console and file
    // auto console_logger = spdlog::stdout_color_mt("console");
    // auto file_logger = spdlog::basic_logger_mt("file_logger", "logs/motor_controller.log");

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("logfile", "motor_controller.txt"));
    auto combined_logger = std::make_shared<spdlog::logger>("name", begin(sinks), end(sinks));
    // register it if you need to access it globally
    spdlog::register_logger(combined_logger);
    combined_logger->flush_on(spdlog::level::debug);

    // Optionally, set the global logger to the console logger
    // spdlog::set_default_logger(console_logger);

    // Set the log level to warn
    spdlog::set_level(spdlog::level::warn);

    // Log messages will be sent to both console and file
    spdlog::set_pattern("[%H:%M:%S %z] [%^%L%$] [thread %t] %v");
    spdlog::flush_on(spdlog::level::warn);

    combined_logger->log(spdlog::level::info, "This is an info message");

    const std::string motor_config_path = "src/motor_control/ethercat_device_configurator/config/motors.yaml";
    MotorConfigLoader motor_loader(motor_config_path);

    std::vector<Motor> loaded_motors = motor_loader.loadDeviceConfigs();

    MotorController motorController(loaded_motors);

    // Set and print positions for motors and gripper
    try
    {
        motorController.setMotorTargetPosition(0, 200.0f); // Out of range, should log a warning
        motorController.setGripperPosition(10.0f);

        for (size_t i = 0; i < motorController.getJoints().size(); ++i)
        {
            std::cout << motorController.getMotors()[i] << std::endl;
        }

        std::cout << motorController.getGripper() << std::endl;

        // Get and print a motor by name
        Motor motor = motorController.getMotor("j2");
        std::cout << "Retrieved Motor: " << motor << std::endl;
        Motor motor1 = motorController.getMotor("Motor3");
    }
    catch (const std::out_of_range &e)
    {
        spdlog::error("Error: {}", e.what());
    }
    catch (const std::invalid_argument &e)
    {
        spdlog::error("Error: {}", e.what());
    }

    return 0;
}
