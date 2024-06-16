#include "tank_firmware/tank_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace tank_firmware
{
TankInterface::TankInterface()
{
}


TankInterface::~TankInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("TankInterface"),
                          "--> Setup Kommunikation zum Port Fehlgeschlagen <-- " << port_);
    }
  }
}


CallbackReturn TankInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
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
    RCLCPP_FATAL(rclcpp::get_logger("TankInterface"), "--> Kein Serial Port Angegeben <--");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> TankInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interface
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> TankInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interface
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn TankInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("TankInterface"), "--> Starte Roboter Hardware ... <--");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0, 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("TankInterface"),
                        "--> Komunikation zum Port Fehlgeschlagen in on_activate() <-- " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("TankInterface"),
              "--> Hardware wurde gestartet und wartet auf Befehle <--");
  return CallbackReturn::SUCCESS;
}


CallbackReturn TankInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("TankInterface"), "--> Verbindung zur Hardware wird beendet!  <--");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("TankInterface"),
                          "--> Komunikation zum Port Fehlgeschlagen in on_deactivate() <-- " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("TankInterface"), "Hardware Verbindung beendet");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type TankInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string
  if(arduino_.IsDataAvailable())
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
      else if(res.at(0) == 't')
      {
        velocity_states_.at(2) = multiplier * std::stod(res.substr(2, res.size()));
      }
      else if(res.at(0) == 'g')
      {
        velocity_states_.at(3) = multiplier * std::stod(res.substr(2, res.size()));
      }
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type TankInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Implement communication protocol with the Arduino
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  char turret_sign = velocity_commands_.at(2) >= 0 ? 'p' : 'n';
  char pipe_sign = velocity_commands_.at(3) >= 0 ? 'p' : 'n';
  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";
  std::string compensate_zeros_turret = "";
  std::string compensate_zeros_pipe = "";
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
  if(std::abs(velocity_commands_.at(2)) < 10.0)
  {
    compensate_zeros_turret = "0";
  }
  else
  {
    compensate_zeros_turret = "";
  }
  if(std::abs(velocity_commands_.at(3)) < 10.0)
  {
    compensate_zeros_pipe = "0";
  }
  else
  {
    compensate_zeros_pipe = "";
  }
  
  message_stream << std::fixed << std::setprecision(4) << 
    "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
    ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << 
    ",t" <<  turret_sign << compensate_zeros_turret << std::abs(velocity_commands_.at(2)) << 
    ",g" <<  pipe_sign << compensate_zeros_pipe << std::abs(velocity_commands_.at(3)) << ",";

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TankInterface"),
                        "--> Komunikation mit dem Arduino wurde Unterbrochen  !! <--"
                            << message_stream.str() << " auf dem Port: " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace tank_firmware

PLUGINLIB_EXPORT_CLASS(tank_firmware::TankInterface, hardware_interface::SystemInterface)