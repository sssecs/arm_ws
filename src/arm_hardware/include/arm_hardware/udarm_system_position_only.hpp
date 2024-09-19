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

#ifndef ARM_HARDWARE__UDARMSYSTEMPOSITIONONLY_HPP_
#define ARM_HARDWARE__UDARMSYSTEMPOSITIONONLY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "scservo_driver/SCServo.h"

#define pi 3.1415926

namespace arm_hardware
{

  struct JointParams
  {
    std::string joint_name;
    double pos_cmd;
    double pos_act;
    double velo_act;
  };

  class UDArmSystemPositionOnlyHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UDArmSystemPositionOnlyHardware);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::string serial_port_;
    int baud_rate_;

    // Store the command for the simulated robot
    std::vector<JointParams> joints_params_;

    uint8_t ID_[4] = {1, 2, 3, 4};
    uint8_t rxPacket_[4];

    s16 Position_[4];
    u16 Speed_[4] = {2400, 2400, 2400, 2400};
    u8 ACC_[4] = {50, 50, 50, 50};

    SMS_STS servo_comm_;

    double pos_test_[3] = {0, 0, 0};
  };

} // namespace arm_hardware

#endif // ARM_HARDWARE__RRBOT_HPP_
