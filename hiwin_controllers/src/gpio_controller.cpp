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

#include "hiwin_controllers/gpio_controller.hpp"

namespace hiwin_controllers
{

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
};

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
};

controller_interface::CallbackReturn GPIOController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<gpio_controller_parameters::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error reading parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn GPIOController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  for (const auto& gpio : params_.gpios)
  {
    const auto& cmd_ifaces = params_.command_interfaces.gpios_map.at(gpio).interfaces;
    for (const auto& iface : cmd_ifaces)
    {
      command_interface_names_.push_back(gpio + "/" + iface);
    }
    const auto& state_ifaces = params_.state_interfaces.gpios_map.at(gpio).interfaces;
    for (const auto& iface : state_ifaces)
    {
      state_interface_names_.push_back(gpio + "/" + iface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn GPIOController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  setup_state_interface_map();
  setup_command_interface_map();

  const auto logger = get_node()->get_logger();

  try
  {
    system_io_pub_ = get_node()->create_publisher<hiwin_msgs::msg::SystemIOStates>("~/system_io_states",
                                                                                   rclcpp::SystemDefaultsQoS());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Failed to create system_io_states publisher: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try
  {
    digital_io_pub_ = get_node()->create_publisher<hiwin_msgs::msg::DigitalIOStates>("~/digital_io_states",
                                                                                     rclcpp::SystemDefaultsQoS());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Failed to create digital_io_states publisher: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try
  {
    set_io_srv_ = get_node()->create_service<hiwin_msgs::srv::SetIO>(
        "~/set_io",
        std::bind(&GPIOController::handle_set_gpio_command, this, std::placeholders::_1, std::placeholders::_2));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Failed to create set_io service: %s", e.what());
    return CallbackReturn::ERROR;
  }

  watchdog_iface_ = {};
  for (auto& iface : command_interfaces_)
  {
    if (iface.get_name() == "system/watchdog")
    {
      watchdog_iface_ = iface;
      last_toggle_time_ = get_node()->now();
      break;
    }
  }

  if (!watchdog_iface_.has_value())
  {
    RCLCPP_WARN(logger, "Watchdog command interface 'system/watchdog' not found. Watchdog may not function.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn GPIOController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::return_type GPIOController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  const auto now = get_node()->now();
  const auto logger = get_node()->get_logger();

  // Update watchdog output every 0.5s
  if (watchdog_iface_)
  {
    if ((time - last_toggle_time_) >= rclcpp::Duration::from_seconds(0.5))
    {
      watchdog_output_state_ = !watchdog_output_state_;
      watchdog_iface_->get().set_value(watchdog_output_state_ ? 1.0 : 0.0);
      last_toggle_time_ = time;
    }
  }

  // Read state interfaces and update internal message
  std::vector<std::string> di_names;
  std::vector<bool> di_values;

  for (auto& iface : state_interfaces_)
  {
    const std::string& full_name = iface.get_name();
    const double value = iface.get_value();

    auto it = cabinet_signal_setters_.find(full_name);
    if (it != cabinet_signal_setters_.end())
    {
      it->second(static_cast<bool>(value));
      continue;
    }

    if (full_name.rfind("digital/", 0) == 0)
    {
      std::string name = full_name.substr(std::string("digital/").length());
      di_names.push_back(name);
      di_values.push_back(!static_cast<bool>(value));
    }
  }

  system_io_msg_.stamp = now;
  system_io_pub_->publish(system_io_msg_);

  digital_io_msg_.stamp = now;
  digital_io_msg_.di_names = di_names;
  digital_io_msg_.di_values = di_values;
  digital_io_pub_->publish(digital_io_msg_);

  // Handle pulse expiration (auto-reset outputs)
  for (auto it = pulse_expirations_.begin(); it != pulse_expirations_.end();)
  {
    if (now >= it->second)
    {
      auto cmd_it = command_map_.find(it->first);
      if (cmd_it != command_map_.end())
      {
        RCLCPP_INFO(logger, "Auto-reset (update): %s", it->first.c_str());
        cmd_it->second.get().set_value(0.0);
      }
      it = pulse_expirations_.erase(it);  // erase and advance
    }
    else
    {
      ++it;
    }
  }

  return controller_interface::return_type::OK;
};

void GPIOController::handle_set_gpio_command(const std::shared_ptr<hiwin_msgs::srv::SetIO::Request> request,
                                             std::shared_ptr<hiwin_msgs::srv::SetIO::Response> response)
{
  std::string full_name = request->io_group + "/" + request->interface_name;
  auto it = command_map_.find(full_name);

  if (it != command_map_.end())
  {
    it->second.get().set_value(request->value);
    response->success = true;
    response->message = "Command set successfully.";

    if (request->value == 1.0 && (full_name == "system/reset_safety_rly" || full_name == "system/reset_driver"))
    {
      pulse_expirations_[full_name] = get_node()->now() + rclcpp::Duration::from_seconds(1.0);
    }
  }
  else
  {
    response->success = false;
    response->message = "Interface not found: " + full_name;
  }
}

void GPIOController::setup_state_interface_map()
{
  cabinet_signal_setters_ = {
    { "system/breaker", [&](bool state) { system_io_msg_.breaker = state; } },
    { "system/e_stop", [&](bool state) { system_io_msg_.e_stop = state; } },
    { "system/e_stop2", [&](bool state) { system_io_msg_.e_stop2 = state; } },
    { "system/fan_error", [&](bool state) { system_io_msg_.fan_error = state; } },
    { "system/capacitor_error", [&](bool state) { system_io_msg_.capacitor_error = state; } },
    { "system/clear_error_notify", [&](bool state) { system_io_msg_.clear_error_notify = state; } },
    { "system/shutdown_notify", [&](bool state) { system_io_msg_.shutdown_notify = state; } },
  };
}

void GPIOController::setup_command_interface_map()
{
  command_map_.clear();
  for (auto& iface : command_interfaces_)
  {
    command_map_.emplace(iface.get_name(), std::ref(iface));
  }
}

}  // namespace hiwin_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hiwin_controllers::GPIOController, controller_interface::ControllerInterface)