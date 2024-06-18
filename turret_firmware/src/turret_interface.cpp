#include "turret_firmware/turret_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace turret_firmware
{

    TurretInterface::TurretInterface()
    {
    }

    TurretInterface::~TurretInterface()
    {
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("TurretInterface"), "Setup Kommunikation zum Port Fehlgeschlagen " << port_);
            }
        }
    }

    CallbackReturn TurretInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
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
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("TurretInterface"), "Kein Serial Port Angegeben ");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_state_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> TurretInterface::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> TurretInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                                                                                 hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn TurretInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("TurretInterface"), "--> Starte Roboter Turret Hardware <--");
        velocity_commands_ = {0.0};

        position_state_ = {0.0};

        velocity_states_ = {0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("TurretInterface"), "Komunikation zum Port Fehlgeschlagen in on_activate() " << port_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("TurretInterface"), "--> Hardware Turret wurde gestartet und wartet auf Befehle <--");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn TurretInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("TurretInterface"), "--> Verbindung zur Turret Hardware wurde beendet!  <--");
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("TurretInterface"), "Komunikation zum Port Fehlgeschlagen in on_deactivate() " << port_);
                return CallbackReturn::FAILURE;
            }
        }
    }

    hardware_interface::return_type TurretInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
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
                if(res.at(0) == 't')
                {
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                }
                
            }
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("TurretInterface"), " Empfange " << message);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TurretInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::stringstream message_stream;
        char turret_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_turret = "";

        if(std::abs(velocity_commands_.at(0)) < 10.0)
        {
            compensate_zeros_turret = "0";
        }
        else
        {
            compensate_zeros_turret = "";
        }
        

        message_stream << std::fixed << std::setprecision(1) << "t" << turret_sign << compensate_zeros_turret << std::abs(velocity_commands_.at(0)) << ",";

        try
        {
            arduino_.Write(message_stream.str());
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("TurretInterface"), " Sende " << message_stream.str());
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("TurretInterface"), "--> Komunikation mit dem Turret Arduino wurde Unterbrochen  !! <-- " << message_stream.str() << " auf dem Port: " << port_);
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(turret_firmware::TurretInterface, hardware_interface::SystemInterface);