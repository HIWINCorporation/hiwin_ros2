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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}

#include "rclcpp/rclcpp.hpp"

#include "hiwin_driver/smbus_gpio_handler.hpp"

SMBusGPIOHandler::SMBusGPIOHandler() : fd_(-1), initialized_(false), input_(8, false), output_(8, false)
{
}

SMBusGPIOHandler::~SMBusGPIOHandler()
{
  close_fd();
}

bool SMBusGPIOHandler::init(const std::string& device_path, int slave_addr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  close_fd();

  fd_ = open(device_path.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("SMBusGPIOHandler"), "Failed to open SMBus device: %s", device_path.c_str());
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, slave_addr) < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("SMBusGPIOHandler"), "Failed to set SMBus slave address");
    close_fd();
    return false;
  }

  initialized_ = true;
  return true;
}

bool SMBusGPIOHandler::is_initialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

void SMBusGPIOHandler::close_fd()
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  initialized_ = false;
}

void SMBusGPIOHandler::update()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_)
    return;

  if (!read_gpio_locked())
    RCLCPP_WARN(rclcpp::get_logger("SMBusGPIOHandler"), "Failed to read GPIO");
  if (!write_gpio_locked())
    RCLCPP_WARN(rclcpp::get_logger("SMBusGPIOHandler"), "Failed to write GPIO");
}

bool SMBusGPIOHandler::get_input(int pin, bool& value) const
{
  if (pin < 0 || pin >= static_cast<int>(input_.size()))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  value = input_.at(pin);
  return true;
}

bool SMBusGPIOHandler::set_output(int pin, bool value)
{
  if (pin < 0 || pin >= static_cast<int>(output_.size()))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  output_.at(pin) = value;
  return true;
}

bool SMBusGPIOHandler::get_input_group(uint8_t group, uint8_t& value) const
{
  if (group != 0 && group >= (input_.size() / 8))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  value = 0x00;
  for (int i = 0; i < 8; ++i)
    value |= input_[i + group * 8] ? 1 << i : 0;

  return true;
}

bool SMBusGPIOHandler::set_output_group(uint8_t group, uint8_t value)
{
  if (group != 0 && group >= (output_.size() / 8))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < 8; ++i)
    output_[i + group * 8] = (value >> i) & 0x01;

  return true;
}

bool SMBusGPIOHandler::read_gpio_locked()
{
  if (fd_ < 0)
    return false;

  int group1 = i2c_smbus_read_byte_data(fd_, 0x00);
  int group2 = i2c_smbus_read_byte_data(fd_, 0x01);

  if (group1 < 0 || group2 < 0)
    return false;

  uint16_t combined = static_cast<uint8_t>(group1) | (static_cast<uint8_t>(group2) << 4);

  for (int i = 0; i < 8; ++i)
    input_[i] = ((combined >> i) & 0x01) ? true : false;

  return true;
}

bool SMBusGPIOHandler::write_gpio_locked()
{
  if (fd_ < 0)
    return false;

  uint8_t group1 = 0;
  for (int i = 0; i < 4; ++i)
    group1 |= (output_[i] ? 1 : 0) << (i + 4);

  uint8_t group2 = 0;
  for (int i = 0; i < 4; ++i)
    group2 |= (output_[i + 4] ? 1 : 0) << (i + 4);

  if (i2c_smbus_write_byte_data(fd_, 0x02, group1) < 0)
    return false;
  if (i2c_smbus_write_byte_data(fd_, 0x03, group2) < 0)
    return false;

  return true;
}
