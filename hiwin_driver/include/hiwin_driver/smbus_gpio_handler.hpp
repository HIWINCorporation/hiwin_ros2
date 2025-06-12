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

#ifndef HIWIN_DRIVER_SMBUS_GPIO_HANDLER_HPP_
#define HIWIN_DRIVER_SMBUS_GPIO_HANDLER_HPP_

#include <vector>
#include <mutex>
#include <string>

#include "hiwin_driver/device_handler.hpp"

class SMBusGPIOHandler : public IDeviceHandler
{
public:
  SMBusGPIOHandler();
  ~SMBusGPIOHandler();

  bool init(const std::string& device_path, int slave_addr);
  bool is_initialized() const;

  void update() override;

  bool get_input(int pin, bool& value) const;
  bool set_output(int pin, bool value);
  bool get_input_group(uint8_t group, uint8_t& value) const;
  bool set_output_group(uint8_t group, uint8_t value);

private:
  bool read_gpio_locked();
  bool write_gpio_locked();
  void close_fd();

  int fd_;
  bool initialized_;
  mutable std::mutex mutex_;
  std::vector<bool> input_;
  std::vector<bool> output_;
};

#endif  // HIWIN_DRIVER_SMBUS_GPIO_HANDLER_HPP_
