#include "../include/solo12_hardware/solo12_hardware.hpp"
#include <string>
#include <vector>

namespace solo12_hardware
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_efforts_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);
  joint_effort_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["effort"])
  {
    state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }
  /*
  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  for (const auto & joint_name : joint_interfaces["effort"])
  {
    command_interfaces.emplace_back(joint_name, "effort", &joint_effort_command_[ind++]);
  }
  */

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // set sensor_states_ values from subscriber
  // responsible for updating the data values of the state_interfaces.
  // (read hardware values for state_interfaces, 
  // e.g. joint encoders and sensor readings)
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // responsible for updating the data values of the command_interfaces
  // sends command interface values to hardware
  // For eg. if hardware support setting a joint velcoity via TCP, then this
  // method accesses data of the corresponding command_interface and
  // sends a packet with the value
  return return_type::OK;
}

}  // namespace solo12_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  solo12_hardware::RobotSystem, hardware_interface::SystemInterface)
