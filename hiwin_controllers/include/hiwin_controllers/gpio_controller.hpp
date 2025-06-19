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

#ifndef HIWIN_CONTROLLERS_GPIO_CONTROLLER_HPP_
#define HIWIN_CONTROLLERS_GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "hiwin_controllers/gpio_controller_parameters.hpp"
#include "hiwin_msgs/msg/system_io_states.hpp"
#include "hiwin_msgs/msg/digital_io_states.hpp"
#include "hiwin_msgs/srv/set_io.hpp"

namespace hiwin_controllers
{

class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const final;

  controller_interface::InterfaceConfiguration state_interface_configuration() const final;

  CallbackReturn on_init() final;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) final;

private:
  void setup_state_interface_map();

  void setup_command_interface_map();

  void handle_set_gpio_command(const std::shared_ptr<hiwin_msgs::srv::SetIO::Request> request,
                               std::shared_ptr<hiwin_msgs::srv::SetIO::Response> response);

  rclcpp::Service<hiwin_msgs::srv::SetIO>::SharedPtr set_io_srv_;

  std::shared_ptr<rclcpp::Publisher<hiwin_msgs::msg::SystemIOStates>> system_io_pub_;
  std::shared_ptr<rclcpp::Publisher<hiwin_msgs::msg::DigitalIOStates>> digital_io_pub_;

  hiwin_msgs::msg::SystemIOStates system_io_msg_;
  hiwin_msgs::msg::DigitalIOStates digital_io_msg_;

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;

  std::shared_ptr<gpio_controller_parameters::ParamListener> param_listener_;
  gpio_controller_parameters::Params params_;

  rclcpp::Time last_toggle_time_;
  bool watchdog_output_state_ = false;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> watchdog_iface_;

  std::unordered_map<std::string, std::function<void(bool)>> cabinet_signal_setters_;
  std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_map_;

  std::unordered_map<std::string, rclcpp::Time> pulse_expirations_;
};

}  // namespace hiwin_controllers

#endif  // HIWIN_DRIVER_GPIO_CONTROLLER_HPP_