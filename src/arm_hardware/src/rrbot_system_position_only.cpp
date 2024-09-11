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

#include "arm_hardware/rrbot_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{
hardware_interface::return_type RRBotSystemPositionOnlyHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }


  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);

  joints_params_.resize(info_.joints.size());

  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Robot has %d joints found. 2 expected.", 
      info_.joints.size());
    return hardware_interface::return_type::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    joints_params_[i].joint_name = info_.joints[i].name;
    joints_params_[i].motor_id = i + 1;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joints_params_[i].joint_name, hardware_interface::HW_IF_POSITION, &joints_params_[i].pos_act));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joints_params_[i].joint_name, hardware_interface::HW_IF_VELOCITY, &joints_params_[i].velo_act));  
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joints_params_[i].joint_name, hardware_interface::HW_IF_POSITION, &joints_params_[i].pos_cmd));
  }

  return command_interfaces;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Starting ...please wait...");

  if(!servo_comm_.begin(baud_rate_, serial_port_.c_str())){
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Faill to start serial port");
        return hardware_interface::return_type::ERROR;
    }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Stopping ...please wait...");

  servo_comm_.end();

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Reading...");

  servo_comm_.syncReadBegin(sizeof(ID_), sizeof(rxPacket_));

  servo_comm_.syncReadPacketTx(ID_, sizeof(ID_), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket_));//同步读指令包发送
		for(uint8_t i=0; i<sizeof(ID_); i++){
			//接收ID[i]同步读返回包
			if(!servo_comm_.syncReadPacketRx(ID_[i], rxPacket_)){
				std::cout<<"ID:"<<(int)ID_[i]<<" sync read error!"<<std::endl;
				continue;//接收解码失败
			}
			joints_params_[i].pos_act_raw = servo_comm_.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			joints_params_[i].velo_act_raw = servo_comm_.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位

      joints_params_[i].pos_act = static_cast<float>(joints_params_[i].pos_act_raw)/4096.0 * 360 - 180;
      joints_params_[i].velo_act = static_cast<float>(joints_params_[i].pos_act_raw)/4096.0 * 360 - 180;

			std::cout<<"ID:"<<int(ID_[i])<<" Position:"<<joints_params_[i].pos_act_raw<<" Speed:"<<joints_params_[i].velo_act_raw<<std::endl;
		}
  
  servo_comm_.syncReadEnd();

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

  for(uint8_t i=0; i<sizeof(ID_); i++)
  {
    Position_[i] = static_cast<int16_t>(joints_params_[i].pos_cmd+180)/360 * 4095;
  }

  servo_comm_.SyncWritePosEx(ID_, sizeof(ID_), Position_, Speed_, ACC_);

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_hardware::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
