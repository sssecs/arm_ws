// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arm_hardware/udarm_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{
  hardware_interface::CallbackReturn UDArmSystemPositionOnlyHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_ = info_.hardware_parameters["serial_port"];
    baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);

    joints_params_.resize(info_.joints.size());

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      joints_params_[i].joint_name = info_.joints[i].name;
      joints_params_[i].pos_cmd = 0;
      joints_params_[i].pos_act = 0;
      joints_params_[i].velo_act = 0;
    }

    if (info_.joints.size() != 3)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
          "Robot has %ld joints found. 3 expected.",
          info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // RRBotSystemPositionOnly has exactly one state and command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UDArmSystemPositionOnlyHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  UDArmSystemPositionOnlyHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_params_[i].pos_act));
    }

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_params_[i].velo_act));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  UDArmSystemPositionOnlyHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_params_[i].pos_cmd));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn UDArmSystemPositionOnlyHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UDArmSystemPositionOnlyHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type UDArmSystemPositionOnlyHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Reading...");

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      joints_params_[i].pos_act = pos_test_[i];
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type UDArmSystemPositionOnlyHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Writing...");

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      pos_test_[i] = joints_params_[i].pos_cmd;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Joints successfully written!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::UDArmSystemPositionOnlyHardware, hardware_interface::SystemInterface)
