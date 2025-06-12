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

#ifndef HIWIN_DRIVER_GPIO_INTERFACE_HPP_
#define HIWIN_DRIVER_GPIO_INTERFACE_HPP_

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "hiwin_driver/smbus_gpio_handler.hpp"
#include "hiwin_driver/serial_io_handler.hpp"

namespace hiwin_driver
{

class HIWINGPIOHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HIWINGPIOHardwareInterface)
  virtual ~HIWINGPIOHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

private:
  void polling_loop();

  void set_input_if_exists(const std::unordered_map<std::string, size_t>& map, const std::string& key,
                           std::vector<double>& output, double value);

  double get_output_if_exists(const std::unordered_map<std::string, size_t>& map, const std::string& key,
                              const std::vector<double>& output);

  std::vector<double> system_input_;
  std::vector<double> system_output_;
  std::vector<double> digital_input_;
  std::vector<double> digital_output_;
  std::vector<double> robot_input_;
  std::vector<double> robot_output_;

  std::unordered_map<std::string, std::unordered_map<std::string, size_t>> command_interface_index_map_;
  std::unordered_map<std::string, std::unordered_map<std::string, size_t>> state_interface_index_map_;

  std::thread polling_thread_;
  std::atomic_bool stop_polling_{ false };
  std::mutex gpio_mutex_;

  std::shared_ptr<SMBusGPIOHandler> smbus_gpio_;
  std::shared_ptr<ISerialProtocol> digital_io_protocol_;
  std::shared_ptr<SerialIOHandler> digital_io_;
  std::shared_ptr<ISerialProtocol> robot_io_protocol_;
  std::shared_ptr<SerialIOHandler> robot_io_;

  std::vector<uint8_t> gpio_in_cache_;
  std::vector<uint8_t> gpio_out_cache_;
  std::vector<uint8_t> digital_in_cache_;
  std::vector<uint8_t> digital_out_cache_;
  std::vector<uint8_t> robot_in_cache_;
  std::vector<uint8_t> robot_out_cache_;
};

}  // namespace hiwin_driver

#endif  // HIWIN_DRIVER_GPIO_INTERFACE_HPP_