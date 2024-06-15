#include "EthercatDeviceConfigurator.hpp"
#include <maxon_epos_ethercat_sdk/Maxon.hpp>

#include "rclcpp/rclcpp.hpp"
#include "hd_interfaces/msg/motor_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <thread>
#include <csignal>
#include <string>
#include <vector>

#define TIME_COUNTDOWN 200ms

using namespace std::chrono_literals;

using std::placeholders::_1;

enum MotorMode
{
    POSITION = 0,
    VELOCITY = 1,
    TORQUE = 2
};

struct MotorCommand
{
    std::string name;
    maxon::Command command;
    std::chrono::steady_clock::time_point command_time;
    double max_velocity = 0;
    double max_torque = 0;
    double pos_lower_limit = 0;
    double pos_upper_limit = 0;
    maxon::ModeOfOperationEnum stationary_mode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;

    MotorCommand(std::string name, maxon::Command command, std::chrono::steady_clock::time_point command_time) : name(name), command(command), command_time(command_time) {}
};

static const double PI = 3.14159265359;
static const double INF = 1e10;
static const std::vector<std::string> DEVICE_NAMES = {"J1", "J2", "J3", "J4", "J5", "J6", "Gripper"};
//static const std::vector<std::string> DEVICE_NAMES = {"J2"};
// static const std::vector<double> MAX_VELOCITIES = {0.4, 0.1, 0.2, 0.6, 0.2, 1, 1, 1, 1, 1}; // {0.2, 0.5, 0.3, 0.3, 0.15, 0.3, 4, 1};    // [rad/s]
// below is used for the control in velocity
// static const std::vector<double> MAX_VELOCITIES = {0.00145, 0.001, 0.001, 0.00145, 0.00145, 0.00145, 1, 1}; // {0.2, 0.5, 0.3, 0.3, 0.15, 0.3, 4, 1};    // [rad/s]
static const std::vector<double> MAX_VELOCITIES = {0.2, 0.13, 0.13, 0.2, 0.2, 0.2, 2}; // {0.2, 0.5, 0.3, 0.3, 0.15, 0.3, 4, 1};    // [rad/s]

static const std::vector<double> MAX_TORQUES = {1, 1, 1, 1, 1, 1, 2, 2, 1, 1};
// static const std::vector<double> POS_LOWER_LIMITS = {-2 * PI, -PI, -PI / 4, -2 * PI, -PI / 2, -PI, -INF, 0, 0};
// static const std::vector<double> POS_UPPER_LIMITS = {2 * PI, PI / 2, PI / 4, 2 * PI, PI / 2, PI, INF, 0, 0};
static const std::vector<double> POS_LOWER_LIMITS = {-2 * PI, -PI, -5 * PI / 6, -2 * PI, -PI / 2, -PI, -INF, 0, 0};
static const std::vector<double> POS_UPPER_LIMITS = {2 * PI, PI / 2, 5 * PI / 6, 2 * PI, PI / 2, PI, INF, 0, 0};

// static const std::vector<double> REDUCTIONS = {-1.0/128, 1.0/2, 1.0, -4.0, 1.0, 1.0/64, 1.0, 1.0};
static const std::vector<double> DIRECTIONS = {1, 1, 1, -1, -1, -1, 1, 1}; // to match directions of MoveIt

static std::vector<bool> should_scan_stationary_states = {true, true, true, true, true, true, true, true, true, true};

static double j5_initial_pos = 0.0;
static double j56_coupling = 1.0;
static bool set_j5_initial_pos = false;

std::vector<MotorCommand> motor_command_list;

std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

void signal_handler(int sig);

class MotorController : public rclcpp::Node
{
public:
    MotorController()
        : Node("MotorController")
    {
        // MATTHIAS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        subscription_velocity_command_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "HD/fsm/joint_vel_cmd", 10, std::bind(&MotorController::manual_direct_command_callback, this, _1));
        subscription_position_command_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "HD/kinematics/joint_pos_cmd", 10, std::bind(&MotorController::position_command_callback, this, _1));
        subscription_single_MotorCommand_ = this->create_subscription<hd_interfaces::msg::MotorCommand>(
            "HD/kinematics/single_joint_cmd", 10, std::bind(&MotorController::motor_command_callback, this, _1));
        subscription_shutdown_ = this->create_subscription<std_msgs::msg::Int8>(
            "ROVER/Maintenance", 10, std::bind(&MotorController::kill, this, _1));
        subscription_set_gripper_torque_ = this->create_subscription<std_msgs::msg::Float64>(
            "HD/kinematics/set_gripper_torque", 10, std::bind(&MotorController::set_gripper_torque, this, _1));
        publisher_state_ = this->create_publisher<sensor_msgs::msg::JointState>("HD/motor_control/joint_telemetry", 10);
        timer_motor_data_ = this->create_wall_timer(
            10ms, std::bind(&MotorController::publish_state, this));
        // MATTHIAS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    }

    static void setup_command_list()
    {
        reorder_command_list();
        for (size_t i = 0; i < motor_command_list.size(); i++)
        {
            motor_command_list[i].max_velocity = MAX_VELOCITIES[i];
            motor_command_list[i].max_torque = MAX_TORQUES[i];
            motor_command_list[i].pos_lower_limit = POS_LOWER_LIMITS[i];
            motor_command_list[i].pos_upper_limit = POS_UPPER_LIMITS[i];
            if (i < 6)
                motor_command_list[i].stationary_mode = maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode;
            else
                motor_command_list[i].stationary_mode = maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode;
        }
    }

    static void set_stationary(size_t motor_index)
    {
        static std::vector<double> stationary_positions = {0, 0, 0, 0, 0, 0, 0, 0};
        auto &command = motor_command_list[motor_index];
        switch (command.stationary_mode)
        {
        case maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode:
            command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
            if (should_scan_stationary_states[motor_index])
            {
                double pos = get_position(motor_index);
                if (weird(motor_index, pos))
                {
                    return;
                }
                stationary_positions[motor_index] = pos;
            }
            command.command.setTargetPosition(stationary_positions[motor_index] * DIRECTIONS[motor_index]);
            break;
        case maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode:
            command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
            command.command.setTargetVelocity(0);
            break;
        case maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode:
            command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
            command.command.setTargetTorque(0);
            break;
            // case maxon::ModeOfOperationEnum::NA:
            //     // TODO Address this case IMPORTANT
            //     break;
        }
        should_scan_stationary_states[motor_index] = false;
    }

private:
    /**                         variablres                             **/
    // commande au moteur
    // std::vector<MotorCommand> motor_command_list;

    // Ros related

    // MATTHIAS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_command_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_position_command_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_set_gripper_torque_;
    rclcpp::Subscription<hd_interfaces::msg::MotorCommand>::SharedPtr subscription_single_MotorCommand_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_shutdown_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_state_;
    // MATTHIAS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    rclcpp::TimerBase::SharedPtr timer_motor_data_;

    // MATTHIAS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    void manual_direct_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "manual_direct_command_callback", msg->data);

        for (uint i = 0; i < motor_command_list.size(); i++)
        {
            // if (i < 6)
            // {
            //     velocity_direct_command(i, msg->data[i]);
            // }
            if (i < 6)
            {
                position_direct_command(i, msg->data[i]);
            }
            else
            {
                torque_direct_command(i, msg->data[i]);
            }
        }
    }

    static bool weird(size_t motor_index, double pos)
    {
        return (pos > POS_UPPER_LIMITS[motor_index] * 1.5 || pos < POS_LOWER_LIMITS[motor_index] * 1.5);
    }

    static void position_direct_command(size_t motor_index, double velocity_scaling_factor)
    {
        std::cout << "Position direct command, motor index: " << motor_index << " | velocity factor: " << velocity_scaling_factor << std::endl;

        // RCLCPP_INFO(this->get_logger(), "setting default velocity %s \n", default_velocity);

        // velocity_scaling_factor in [-1, 1]
        auto now = std::chrono::steady_clock::now();
        static std::vector<std::chrono::steady_clock::time_point> last_cmd_times = {now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN, now - 2 * TIME_COUNTDOWN};
        static std::vector<double> position_commands = {0, 0, 0, 0, 0, 0, 0, 0};

        auto &command = motor_command_list[motor_index];
        command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);

        double velocity = velocity_scaling_factor * command.max_velocity;
        auto time_since_last_cmd = now - last_cmd_times[motor_index];
        // RCLCPP_INFO(this->get_logger(), "Requesting velocity %s \n", position_commands[motor_index]);

        if (time_since_last_cmd < TIME_COUNTDOWN)
        {
            // if previous command is not too old
            const double ms_to_s = 0.001;
            double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_last_cmd).count() * ms_to_s;
            position_commands[motor_index] += dt * velocity;
            // RCLCPP_INFO(this->get_logger(), "setting position command to %s \n", position_commands[motor_index]);
            std::cout << "setting position command to " << position_commands[motor_index] << std::endl;

            command.command.setTargetPosition(position_commands[motor_index] * DIRECTIONS[motor_index]);
        }
        else
        { // scanning
            double pos = get_position(motor_index);
            if (weird(motor_index, pos))
            {
                return;
            }
            position_commands[motor_index] = pos;
        }
        last_cmd_times[motor_index] = now;
        command.command_time = now;
        should_scan_stationary_states[motor_index] = true;
        enforce_limits(motor_index);
    }

    static void torque_direct_command(size_t motor_index, double torque_scaling_factor)
    {
        // torque_scaling_factor in [-1, 1]
        auto &command = motor_command_list[motor_index];
        double torque = torque_scaling_factor * command.max_torque * DIRECTIONS[motor_index];
        command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
        command.command.setTargetTorque(torque);
        command.command_time = std::chrono::steady_clock::now();
        should_scan_stationary_states[motor_index] = true;
        enforce_limits(motor_index);
    }

    static void velocity_direct_command(size_t motor_index, double velocity_scaling_factor)
    {
        // velocity_scaling_factor in [-1, 1]
        auto &command = motor_command_list[motor_index];
        double velocity = velocity_scaling_factor * command.max_velocity * DIRECTIONS[motor_index];
        std::cout << "Velocity direct command" << std::endl;
        std::cout << " scaling factor: " << velocity_scaling_factor << "\n";
        std::cout << "  requested velicity: " << velocity << std::endl;
        command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
        command.command.setTargetVelocity(velocity);
        command.command_time = std::chrono::steady_clock::now();
        should_scan_stationary_states[motor_index] = true;
        enforce_limits(motor_index);
    }

    void position_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "position_command_callback");

        double j5_pos = 0.0; // for decoupling of j5 and j6
        bool j6_on_hall = false;

        for (uint i = 0; i < 6; i++) // only accepting position commands for j1-6
        {
            if (i == 50)
            { // check if i = 50
                motor_command_list[i].command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                motor_command_list[i].command_time = std::chrono::steady_clock::now();
                should_scan_stationary_states[i] = true;
            }
            else
            {
                motor_command_list[i].command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                double new_pos = msg->data[i] * DIRECTIONS[i];
                if (j6_on_hall)
                {
                    if (i == 4)
                        j5_pos = new_pos;
                    else if (i == 5)
                    {
                        double j5_delta = j5_pos - j5_initial_pos;
                        new_pos += j56_coupling * j5_delta;
                    }
                }
                // new_pos = std::min(std::max(new_pos, POS_LOWER_LIMITS[i]), POS_UPPER_LIMITS[i]);
                if (new_pos < POS_LOWER_LIMITS[i])
                    new_pos = POS_LOWER_LIMITS[i];
                if (new_pos > POS_UPPER_LIMITS[i])
                    new_pos = POS_UPPER_LIMITS[i];
                motor_command_list[i].command.setTargetPosition(new_pos);
                motor_command_list[i].command_time = std::chrono::steady_clock::now();
                should_scan_stationary_states[i] = true;
                enforce_limits(i);
            }
        }
    }

    void set_gripper_torque(const std_msgs::msg::Float64::SharedPtr msg)
    {
        for (auto &motor_command : motor_command_list)
        {
            if (motor_command.name == "Gripper")
                motor_command.max_torque = msg->data;
        }
    }

    void publish_state()
    {

        double j5_pos = 0.0; // for decoupling of j5 and j6
        bool j6_on_hall = false;

        sensor_msgs::msg::JointState msg;
        for (size_t i = 0; i < motor_command_list.size(); i++)
        {
            // std::cout << "   about to publish state of " << DEVICE_NAMES[i] << std::endl;
            // if (DEVICE_NAMES[i] != "J6")
            // {
            //     continue;
            // }
            auto slave = configurator->getSlave(DEVICE_NAMES[i]);
            // std::cout << "   Done to publish state of " << DEVICE_NAMES[i] << std::endl;

            std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

            msg.name.push_back(slave->getName());

            auto getReading = maxon_slave_ptr->getReading();
            double pos = getReading.getActualPosition() * DIRECTIONS[i];
            if (j6_on_hall)
            {
                if (i == 4)
                {
                    j5_pos = pos;
                    if (!set_j5_initial_pos)
                    {
                        j5_initial_pos = j5_pos;
                        set_j5_initial_pos = true;
                    }
                }
                else if (i == 5)
                {
                    double j5_delta = j5_pos - j5_initial_pos;
                    pos -= j56_coupling * j5_delta;
                }
            }
            msg.position.push_back(pos);
            msg.velocity.push_back(getReading.getActualVelocity() * DIRECTIONS[i]);
            msg.effort.push_back(getReading.getActualCurrent() * DIRECTIONS[i]);
        }
        publisher_state_->publish(msg);
    }

    void kill(const std_msgs::msg::Int8::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "kill_command_callback");

        static const int LAUNCH = 1;
        static const int ABORT = 2;
        static const int WAIT = 3;
        static const int RESUME = 4;
        static const int CANCEL = 5;
        switch (msg->data)
        {
        case ABORT:
        case CANCEL:
            signal_handler(0);
            rclcpp::shutdown();
            break;
        }
    }
    // MATTHIAS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    void motor_command_callback(const hd_interfaces::msg::MotorCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "motor_command_callback");

        // TODO: add the multiplication by the direction of the corresponding joint in this function
        for (uint i = 0; i < motor_command_list.size(); i++)
        {
            auto &motor_command = motor_command_list[i];

            if (motor_command.name == msg->name)
            {
                switch (msg->mode)
                {
                case MotorMode::POSITION:
                    motor_command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode);
                    motor_command.command.setTargetPosition(msg->command);
                    break;
                case MotorMode::VELOCITY:
                    motor_command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
                    motor_command.command.setTargetVelocity(msg->command * motor_command.max_velocity * DIRECTIONS[i]);
                    break;
                case MotorMode::TORQUE:
                    motor_command.command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    motor_command.command.setTargetTorque(msg->command * motor_command.max_torque * DIRECTIONS[i]);
                    break;
                default:
                    std::cerr << "Motor mode not recognized" << std::endl;
                    break;
                }
                motor_command.command_time = std::chrono::steady_clock::now();
                should_scan_stationary_states[i] = true;
                enforce_limits(i);

                return;
            }
        }
    }

    // MATTHIAS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    static double get_position(size_t motor_index)
    {
        auto slave = configurator->getSlave(DEVICE_NAMES[motor_index]);
        std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);
        auto getReading = maxon_slave_ptr->getReading();
        return getReading.getActualPosition() * DIRECTIONS[motor_index];
    }

    static double get_target_position(size_t motor_index)
    {
        return motor_command_list[motor_index].command.getTargetPosition();
    }

    static void reorder_command_list()
    {
        // sorting motor_command_list by real order of motors in the arm
        for (size_t i = 0; i < motor_command_list.size(); i++)
        {
            for (size_t j = i; j < motor_command_list.size(); j++)
            {
                if (motor_command_list[j].name == DEVICE_NAMES[i])
                {
                    if (i != j)
                    {
                        auto temp = motor_command_list[i];
                        motor_command_list[i] = motor_command_list[j];
                        motor_command_list[j] = temp;
                    }
                    break;
                }
            }
        }
    }

    static void enforce_limits(size_t motor_index)
    {
        size_t i = motor_index;
        auto &command = motor_command_list[i];
        switch (command.command.getModeOfOperation())
        {
        case maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode:
        {
            double pos = get_target_position(i);
            if (pos < POS_LOWER_LIMITS[i])
                command.command.setTargetPosition(POS_LOWER_LIMITS[i]);
            if (pos > POS_UPPER_LIMITS[i])
                command.command.setTargetPosition(POS_UPPER_LIMITS[i]);
            break;
        }
        case maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode:
            // Not Implemented
            break;
        case maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode:
            // Not Implemented
            break;
        }
    }
    // MATTHIAS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
};

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
        for (size_t i = 0; i < motor_command_list.size(); i++)
        {
            auto &motor_command = motor_command_list[i];

            // Keep constant update rate
            // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

            auto slave = configurator->getSlave(motor_command.name);

            std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

            if (!maxonEnabledAfterStartup)
            {
                // Set maxons to operation enabled state, do not block the call!
                maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
            }

            // set commands if we can
            if (maxon_slave_ptr->lastPdoStateChangeSuccessful() &&
                maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
            {
                if (std::chrono::steady_clock::now() - motor_command.command_time >= TIME_COUNTDOWN)
                {

                    MotorController::set_stationary(i);
                }
                maxon_slave_ptr->stageCommand(motor_command.command);
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
        counter++;
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

#include <unistd.h>
#include <stdio.h>
#include <limits.h>

/*
spin file as first command line argument.
 */
int main(int argc, char **argv)
{
    std::cout << "=== Starting motor control node ===" << std::endl;
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);

    // if (argc < 2)
    // {
    //     std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
    //     return EXIT_FAILURE;
    // }
    // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)

    // char *path = "src/motor_control/ethercat_device_configurator/config_mot/kerby_setup.yaml"; // kerby setup
    char *path = "motor_control/ethercat_device_configurator/config_mot/onyx_setup.yaml"; // onyx setup

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

    std::cout << "=== Setting up motor command list ===" << std::endl;
    maxon::Command command;
    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
    command.setTargetTorque(0.0);

    std::cout << "=== Setting commands to slaves ===" << std::endl;
    for (auto &slave : configurator->getSlaves())
    {
        std::cout << "   Slave: " << slave->getName() << std::endl;
        motor_command_list.push_back(MotorCommand({slave->getName(), command, std::chrono::steady_clock::now()}));
        std::cout << slave->getName() << std::endl;
    }
    MotorController::setup_command_list();

    for (auto &motor_command : motor_command_list)
    {
        std::cout << "   " << motor_command.name << std::endl;
    }

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

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorController>();

    std::cout << "Node finished" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}