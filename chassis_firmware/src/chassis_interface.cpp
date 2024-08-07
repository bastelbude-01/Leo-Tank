#include "chassis_firmware/chassis_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace chassis_firmware
{

    ChassisInterface::ChassisInterface()
    {
    }

    ChassisInterface::~ChassisInterface()
    {
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("ChassisInterface"), "Setup Kommunikation zum Port Fehlgeschlagen " << port_);
            }
        }
    }

    CallbackReturn ChassisInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ChassisInterface"), "Kein Serial Port Angegeben ");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_state_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ChassisInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                                                                             hardware_interface::HW_IF_POSITION, &position_state_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                                                                             hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ChassisInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                                                                                 hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn ChassisInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ChassisInterface"), "--> Starte Roboter Chassis Hardware <--");
        velocity_commands_ = {0.0,
                              0.0};

        position_state_ = {0.0,
                           0.0};

        velocity_states_ = {0.0,
                            0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ChassisInterface"), "Komunikation zum Port Fehlgeschlagen in on_activate() " << port_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("ChassisInterface"), "--> Hardware Chassis wurde gestartet und wartet auf Befehle <--");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ChassisInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ChassisInterface"), "--> Verbindung zur Chassis Hardware wurde beendet!  <--");
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("ChassisInterface"), "Komunikation zum Port Fehlgeschlagen in on_deactivate() " << port_);
                return CallbackReturn::FAILURE;
            }
        }
    }

    hardware_interface::return_type ChassisInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (arduino_.IsDataAvailable())
        {
            std::string message;
            arduino_.ReadLine(message);
            std::stringstream ss(message);
            std::string res;
            int multiplier = 1;
            while(std::getline(ss, res, ','))
            {
                multiplier = res.at(1) == 'p' ? 1 : -1;
                if(res.at(0) == 'r')
                {
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                }
                else if(res.at(0) == 'l')
                {
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                }
            }
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("ChassisInterface"), " Empfange " << message);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ChassisInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands_.at(0)) < 10.0)
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }
        if(std::abs(velocity_commands_.at(1)) < 10.0)
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0))
                       << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            arduino_.Write(message_stream.str());
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("ChassisInterface"), " Sende " << message_stream.str());
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ChassisInterface"), "--> Komunikation mit dem Chassis Arduino wurde Unterbrochen  !! <-- " << message_stream.str() << " auf dem Port: " << port_);
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(chassis_firmware::ChassisInterface, hardware_interface::SystemInterface);