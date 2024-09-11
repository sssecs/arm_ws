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

#ifndef ARM_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
#define ARM_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "arm_hardware/visibility_control.h"

#include "scservo_driver/SCServo.h"

namespace arm_hardware
{

struct JointParams
{
  std::string joint_name;
  u8 motor_id;
  s16 pos_cmd_raw;
  s16 pos_act_raw;
  s16 velo_act_raw;
  double pos_cmd;
  double pos_act;
  double velo_act;
};

class RRBotSystemPositionOnlyHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware)

  ARM_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ARM_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ARM_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ARM_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  ARM_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  ARM_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ARM_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::string serial_port_;
  int baud_rate_;

  // Store the command for the simulated robot
  std::vector<JointParams> joints_params_;

  uint8_t ID_[2] = {1, 2};
  uint8_t rxPacket_[4];

  s16 Position_[2];
  u16 Speed_[2] = {2400, 2400};
  u8 ACC_[2] = {50, 50};

  SMS_STS servo_comm_;
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
