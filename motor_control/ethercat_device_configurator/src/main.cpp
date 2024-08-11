#include "spdlog/sinks/stdout_color_sinks.h"
#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <vector>
#include "MotorController.h"
#include "MotorConfigLoader.h"
#include "rclcpp/rclcpp.hpp"
#include "MotorNode.hpp"

#include "EthercatDeviceConfigurator.hpp"
#include <maxon_epos_ethercat_sdk/Maxon.hpp>


EthercatDeviceConfigurator::SharedPtr configurator;
std::shared_ptr<MotorController> motor_controller;
bool abrt = false;
std::unique_ptr<std::thread> worker_thread;
#define TIME_COUNTDOWN 200ms

using namespace std::chrono_literals;



void worker()
{
    bool rtSuccess = true;
    for (const auto &master : configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }
    std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.") << std::endl;

    // Flag to set the drive state for the elmos on first startup
    bool maxonEnabledAfterStartup = false;

    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
    */
    while (!abrt)
    {
        /*
        ** Update each master.
        ** This sends tha last staged commands and reads the latest readings over EtherCAT.
        ** The StandaloneEnforceRate update mode is used.
        ** This means that average update rate will be close to the target rate (if possible).
        */
        for (const auto &master : configurator->getMasters())
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
        }

        /*
        ** Do things with the attached devices.
        ** Your lowlevel control input / measurement logic goes here.
        ** Different logic can be implemented for each device.
        */
        for (Motor& motor : motor_controller->getMotors())
        {

            // Keep constant update rate
            // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

            auto slave = configurator->getSlave(motor.getName());

            std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

            if (!maxonEnabledAfterStartup)
            {
                // Set maxons to operation enabled state, do not block the call!
                maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
            }

            // set commands if we can
            if (maxon_slave_ptr->lastPdoStateChangeSuccessful() 
                )
            {
                maxon::Reading reading = maxon_slave_ptr->getReading();
                // if (std::chrono::steady_clock::now() - motor_command.command_time >= TIME_COUNTDOWN)
                // {

                //     MotorController::set_stationary(i);
                // }
                motor.setReadPosition(reading.getActualPosition());
                motor.setReadVelocity(reading.getActualVelocity());
                motor.setReadTorque(reading.getActualTorque());
                motor.setReadCurrent(reading.getActualCurrent());

                if( reading.getDriveState() == maxon::DriveState::OperationEnabled)
                {   
                    // maxon::Command command = motor.getCommand();
                    // command.setTargetPosition(reading.getActualPosition() + 10);
                    // maxon_slave_ptr->stageCommand(command);
                    maxon_slave_ptr->stageCommand(motor.getCommand());

                }

            }
            else
            {
                MELO_WARN_STREAM("Maxon '" << maxon_slave_ptr->getName()
                                           << "': " << maxon_slave_ptr->getReading().getDriveState());

                if (maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::Fault)
                {
                    // Set maxons to operation enabled state, do not block the call!
                    maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                }
            }

            // Constant update rate
            // std::this_thread::sleep_until(start_time + std::chrono::milliseconds(1));
        }
        maxonEnabledAfterStartup = true;
    }
}

/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
*/
void signal_handler(int sig)
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
    */
    for (const auto &master : configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    worker_thread->join();

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
    */
    for (const auto &master : configurator->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

int main(int argc, char **argv)
{
    // Set up spdlog to log to both console and file
    auto console_logger = spdlog::stdout_color_mt("console");
    auto file_logger = spdlog::basic_logger_mt("file_logger", "logs/motor_controller.log");

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("logfile", "motor_controller.txt"));
    auto combined_logger = std::make_shared<spdlog::logger>("name", begin(sinks), end(sinks));
    // register it if you need to access it globally
    spdlog::register_logger(combined_logger);
    combined_logger->flush_on(spdlog::level::debug);

    // Optionally, set the global logger to the console logger
    spdlog::set_default_logger(console_logger);

    // Set the log level to warn
    spdlog::set_level(spdlog::level::warn);

    // Log messages will be sent to both console and file
    spdlog::set_pattern("[%H:%M:%S %z] [%^%L%$] [thread %t] %v");
    spdlog::flush_on(spdlog::level::warn);

    combined_logger->log(spdlog::level::info, "This is an info message");

    const std::string motor_config_path = "src/motor_control/ethercat_device_configurator/config/motors.yaml";
    


    rclcpp::init(argc, argv);

    std::shared_ptr<MotorNode> node = std::make_shared<MotorNode>(motor_config_path);
    motor_controller = node->getMotorController();
    motor_controller->initializeMotors();
    std::cout << "Node finished" << std::endl;



    std::cout << "=== Starting motor control node ===" << std::endl;
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);


    // char *path = "src/motor_control/ethercat_device_configurator/config_mot/kerby_setup.yaml"; // kerby setup
    const std::string path = "src/motor_control/ethercat_device_configurator/config_mot/onyx_setup.yaml"; // onyx setup

    std::cout << "=== Creating Ethercat Device Configurator ===" << std::endl;
    configurator = std::make_shared<EthercatDeviceConfigurator>(path);

    /*
    ** Start all masters.
    ** There is exactly one bus per master which is also started.
    ** All online (i.e. SDO) configuration is done during this call.
    ** The EtherCAT interface is active afterwards, all drives are in Operational
    ** EtherCAT state and PDO communication may begin.
    */
    std::cout << "=== Starting all masters ===" << std::endl;
    for (auto &master : configurator->getMasters())
    {
        // std::cout << "   Master: " << master->getConfiguration() << std::endl;

        if (!master->startup())
        {
            std::cerr << "Master Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // std::cout << "=== Setting up motor command list ===" << std::endl;
    // maxon::Command command;
    // command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
    // command.setTargetTorque(0.0);

    // std::cout << "=== Setting commands to slaves ===" << std::endl;
    // for (auto &slave : configurator->getSlaves())
    // {
    //     std::cout << "   Slave: " << slave->getName() << std::endl;
    //     motor_command_list.push_back(MotorCommand({slave->getName(), command, std::chrono::steady_clock::now()}));
    //     std::cout << slave->getName() << std::endl;
    // }


    std::cout << "=== Starting worker thread ===" << std::endl;
    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);

    std::cout << "=== Waiting for PDO cycles to pass ===" << std::endl;
    /*
    ** Wait for a few PDO cycles to pass.
    ** Set anydrives into to ControlOp state (internal state machine, not EtherCAT states)
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (auto &slave : configurator->getSlaves())
    {
        std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
    }

    std::cout << "Startup finished" << std::endl;
    // nothing further to do in this thread.


    std::cout << "Node finished" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
