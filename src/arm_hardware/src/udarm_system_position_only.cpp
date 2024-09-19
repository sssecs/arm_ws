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
  void Motor::Init(s16 count_zero, int direction)
  {
    this->count_zero_ = count_zero;
    this->direction_ = direction;
  }

  double Motor::Count2Rad(s16 count_in)
  {
    return static_cast<float>(this->direction_ * (count_in - this->count_zero_)) / 4096.0 * 2 * pi;
  }

  s16 Motor::Rad2Count(double rad_in)
  {
    return (static_cast<int16_t>(rad_in / pi / 2 * 4096) - this->count_zero_) * this->direction_;
  }

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

    motors_.resize(4);

    motors_[0].Init(stoi(info_.hardware_parameters["motor1_zero"]), stoi(info_.hardware_parameters["motor1_direction"]));
    motors_[1].Init(stoi(info_.hardware_parameters["motor2_zero"]), stoi(info_.hardware_parameters["motor2_direction"]));
    motors_[2].Init(stoi(info_.hardware_parameters["motor3_zero"]), stoi(info_.hardware_parameters["motor3_direction"]));
    motors_[3].Init(stoi(info_.hardware_parameters["motor4_zero"]), stoi(info_.hardware_parameters["motor4_direction"]));

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
    if (!servo_comm_.begin(baud_rate_, serial_port_.c_str()))
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("UDArmSystemPositionOnlyHardware"),
          "Faill to start serial port");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn UDArmSystemPositionOnlyHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    servo_comm_.end();
    RCLCPP_INFO(rclcpp::get_logger("UDArmSystemPositionOnlyHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type UDArmSystemPositionOnlyHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    servo_comm_.syncReadBegin(sizeof(ID_), sizeof(rxPacket_));

    servo_comm_.syncReadPacketTx(ID_, sizeof(ID_), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket_)); // 同步读指令包发送
    for (uint8_t i = 0; i < sizeof(ID_); i++)
    {
      // 接收ID[i]同步读返回包
      if (!servo_comm_.syncReadPacketRx(ID_[i], rxPacket_))
      {
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Sync read error, ID : %d", ID_[i]);
        continue;
      }

      count_read_buffer_[i] = servo_comm_.syncReadRxPacketToWrod(15);
      pos_read_buffer_[i] = motors_[i].Count2Rad(count_read_buffer_[i]);

      switch (ID_[i])
      {
      case 1:
        joints_params_[0].pos_act = (pos_read_buffer_[0] + pos_read_buffer_[1]) / 2;
        break;

      case 2:
        break;

      case 3:
        joints_params_[1].pos_act = pos_read_buffer_[2];
        break;

      case 4:
        joints_params_[2].pos_act = pos_read_buffer_[3];
        break;
      }
    }

    servo_comm_.syncReadEnd();

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Pos1 : %f, Pos2 : %f, Pos3 : %f, Pos4: %f, Raw1: %d, Raw2: %d, Raw3: %d, Raw4: %d",
                pos_read_buffer_[0], pos_read_buffer_[1], pos_read_buffer_[2], pos_read_buffer_[3],
                count_read_buffer_[0], count_read_buffer_[1], count_read_buffer_[2], count_read_buffer_[3]);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type UDArmSystemPositionOnlyHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      pos_test_[i] = joints_params_[i].pos_cmd;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::UDArmSystemPositionOnlyHardware, hardware_interface::SystemInterface)
