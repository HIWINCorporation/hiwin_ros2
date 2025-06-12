// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "hiwin_driver/gpio_interface.hpp"

namespace hiwin_driver
{
HIWINGPIOHardwareInterface::~HIWINGPIOHardwareInterface()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn
HIWINGPIOHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = system_info;

  for (const hardware_interface::ComponentInfo& gpio : info_.gpios)
  {
    if (gpio.name == "system")
    {
      if (gpio.command_interfaces.size() != 4)
      {
      }

      if (gpio.state_interfaces.size() != 7)
      {
      }
    }
    else if (gpio.name == "digital")
    {
      int byte_size = 0;
      if (gpio.command_interfaces.size() > 32)
      {
        digital_output_.resize(32, 0.0);
      }
      else
      {
        digital_output_.resize(gpio.command_interfaces.size(), 0.0);
      }
      byte_size = (digital_output_.size() / 8) + ((digital_output_.size() % 8) ? 1 : 0);
      digital_out_cache_.resize(byte_size, 0.0);

      if (gpio.state_interfaces.size() > 32)
      {
        digital_input_.resize(32, 0.0);
      }
      else
      {
        digital_input_.resize(gpio.state_interfaces.size(), 0.0);
      }
      byte_size = (digital_input_.size() / 8) + ((digital_input_.size() % 8) ? 1 : 0);
      digital_in_cache_.resize(byte_size, 0.0);
    }
    else if (gpio.name == "robot")
    {
      int byte_size = 0;
      if (gpio.command_interfaces.size() > 32)
      {
        robot_output_.resize(32, 0.0);
      }
      else
      {
        robot_output_.resize(gpio.command_interfaces.size(), 0.0);
      }
      byte_size = (robot_output_.size() / 8) + ((robot_output_.size() % 8) ? 1 : 0);
      robot_out_cache_.resize(byte_size, 0.0);

      if (gpio.state_interfaces.size() > 32)
      {
        robot_input_.resize(32, 0.0);
      }
      else
      {
        robot_input_.resize(gpio.state_interfaces.size(), 0.0);
      }
      byte_size = (robot_input_.size() / 8) + ((robot_input_.size() % 8) ? 1 : 0);
      robot_in_cache_.resize(byte_size, 0.0);
    }
  }

  system_input_.assign(7, 0);
  system_output_.assign(4, 0);

  gpio_in_cache_.assign(1, 0x00);
  gpio_out_cache_.assign(1, 0x00);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HIWINGPIOHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const hardware_interface::ComponentInfo& gpio : info_.gpios)
  {
    for (size_t i = 0; i < gpio.state_interfaces.size(); ++i)
    {
      const std::string& iface_name = gpio.state_interfaces[i].name;

      state_interface_index_map_[gpio.name][iface_name] = i;

      double* data_ptr = nullptr;
      if (gpio.name == "system")
        data_ptr = &system_input_[i];
      else if (gpio.name == "digital")
        data_ptr = &digital_input_[i];
      else if (gpio.name == "robot")
        data_ptr = &robot_input_[i];

      state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, iface_name, data_ptr));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HIWINGPIOHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const hardware_interface::ComponentInfo& gpio : info_.gpios)
  {
    for (size_t i = 0; i < gpio.command_interfaces.size(); ++i)
    {
      const std::string& iface_name = gpio.command_interfaces[i].name;

      command_interface_index_map_[gpio.name][iface_name] = i;

      double* data_ptr = nullptr;
      if (gpio.name == "system")
        data_ptr = &system_output_[i];
      else if (gpio.name == "digital")
        data_ptr = &digital_output_[i];
      else if (gpio.name == "robot")
        data_ptr = &robot_output_[i];

      command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, iface_name, data_ptr));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
HIWINGPIOHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "Starting ...please wait...");

  smbus_gpio_ = std::make_shared<SMBusGPIOHandler>();

  digital_io_protocol_ = std::make_shared<DigitalIOProtocol>();
  digital_io_ = std::make_shared<SerialIOHandler>(digital_io_protocol_);

  robot_io_protocol_ = std::make_shared<RobotIOProtocol>();
  robot_io_ = std::make_shared<SerialIOHandler>(robot_io_protocol_);

  smbus_gpio_->init("/dev/i2c-0", 0x20);
  digital_io_->init("/dev/ttyS1", B115200);
  // robot_io_->init("/dev/ttyS2", B115200);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HIWINGPIOHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "Activating HW interface");

  stop_polling_ = false;
  polling_thread_ = std::thread(&HIWINGPIOHardwareInterface::polling_loop, this);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HIWINGPIOHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "Stopping ...please wait...");

  stop_polling_ = true;
  if (polling_thread_.joinable())
  {
    polling_thread_.join();
  }

  smbus_gpio_.reset();
  digital_io_protocol_.reset();
  digital_io_.reset();
  robot_io_protocol_.reset();

  RCLCPP_INFO(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HIWINGPIOHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period)
{
  uint8_t gpio_snapshot = 0x00;
  uint8_t digital_io_snapshot = 0x00;

  {
    std::lock_guard<std::mutex> lock(gpio_mutex_);

    for (int i = 0; i < digital_input_.size(); i++)
    {
      digital_io_snapshot = digital_in_cache_.at(i / 8);
      digital_input_[i] = (digital_io_snapshot & (1 << (i % 8))) ? 1.0 : 0.0;
    }

    gpio_snapshot = gpio_in_cache_.at(0);
  }

  auto& idx_map = state_interface_index_map_["system"];
  set_input_if_exists(idx_map, "breaker", system_input_, (gpio_snapshot & 0x01) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "e_stop", system_input_, (gpio_snapshot & 0x02) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "clear_error_notify", system_input_, (gpio_snapshot & 0x04) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "fan_error", system_input_, (gpio_snapshot & 0x08) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "e_stop2", system_input_, (gpio_snapshot & 0x10) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "shutdown_notify", system_input_, (gpio_snapshot & 0x20) ? 1.0 : 0.0);
  set_input_if_exists(idx_map, "capacitor_error", system_input_, (gpio_snapshot & 0x40) ? 1.0 : 0.0);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HIWINGPIOHardwareInterface::write(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  uint8_t gpio = 0x00;
  uint8_t digital_io = 0x00;

  auto& cmd_map = command_interface_index_map_["system"];
  (get_output_if_exists(cmd_map, "watchdog", system_output_) == 1.0) ? gpio |= 0x01 : gpio &= ~0x01;
  (get_output_if_exists(cmd_map, "reset_safety_rly", system_output_) == 1.0) ? gpio |= 0x02 : gpio &= ~0x02;
  (get_output_if_exists(cmd_map, "reset_driver", system_output_) == 1.0) ? gpio |= 0x04 : gpio &= ~0x04;

  std::lock_guard<std::mutex> lock(gpio_mutex_);
  for (int i = 0; i < digital_output_.size(); i++)
  {
    (digital_output_[i] == 1.0) ? digital_io |= (1 << (i % 8)) : digital_io &= ~(1 << (i % 8));
    digital_out_cache_.at(i / 8) = digital_io;
  }
  gpio_out_cache_.at(0) = gpio;

  return hardware_interface::return_type::OK;
}

void HIWINGPIOHardwareInterface::set_input_if_exists(const std::unordered_map<std::string, size_t>& map,
                                                     const std::string& key, std::vector<double>& output, double value)
{
  auto it = map.find(key);
  if (it != map.end())
  {
    output[it->second] = value;
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "State interface index for '%s' not found.",
                key.c_str());
  }
}

double HIWINGPIOHardwareInterface::get_output_if_exists(const std::unordered_map<std::string, size_t>& map,
                                                        const std::string& key, const std::vector<double>& output)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    RCLCPP_WARN(rclcpp::get_logger("HIWINGPIOHardwareInterface"), "Command interface index for '%s' not found.",
                key.c_str());
    return 0.0;
  }
  return output[it->second];
}

void HIWINGPIOHardwareInterface::polling_loop()
{
  std::vector<std::shared_ptr<IDeviceHandler>> handlers;
  handlers.emplace_back(smbus_gpio_);
  handlers.emplace_back(digital_io_);

  rclcpp::Rate rate(20);
  while (!stop_polling_.load())
  {
    {
      std::lock_guard<std::mutex> lock(gpio_mutex_);
      smbus_gpio_->set_output_group(0, gpio_out_cache_.at(0));
      digital_io_->set_output_group(0, digital_out_cache_.at(0));
    }

    for (auto& h : handlers)
    {
      h->update();
    }

    {
      std::lock_guard<std::mutex> lock(gpio_mutex_);
      smbus_gpio_->get_input_group(0, gpio_in_cache_.at(0));
      digital_io_->get_input_group(0, digital_in_cache_.at(0));
    }

    rate.sleep();
  }
}

}  // namespace hiwin_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hiwin_driver::HIWINGPIOHardwareInterface, hardware_interface::SystemInterface)
