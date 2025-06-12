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
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"

#include "hiwin_driver/serial_io_handler.hpp"

SerialIOHandler::SerialIOHandler(std::shared_ptr<ISerialProtocol> protocol)
  : fd_(-1)
  , initialized_(false)
  , protocol_(std::move(protocol))
  , input_(32, false)
  , output_(32, false)
  , output_state_(32, false)
{
  if (!protocol_)
  {
    throw std::runtime_error("ISerialProtocol instance cannot be null.");
  }
}

SerialIOHandler::~SerialIOHandler()
{
  close_fd();
}

bool SerialIOHandler::init(const std::string& device_path, int baudrate)
{
  device_path_ = device_path;
  baudrate_ = baudrate;

  if (!open_port(device_path))
    return false;

  if (!configure_port(baudrate))
  {
    close_fd();
    return false;
  }

  initialized_ = true;
  return true;
}

bool SerialIOHandler::open_port(const std::string& device_path)
{
  fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("SerialIOHandler"), "Failed to open serial device: %s", device_path.c_str());
    return false;
  }
  return true;
}

bool SerialIOHandler::configure_port(int baudrate)
{
  struct termios options;
  if (tcgetattr(fd_, &options) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("SerialIOHandler"), "Failed to get attributes:  %s",
                std::string(strerror(errno)).c_str());
    return false;
  }

  cfsetospeed(&options, baudrate);
  cfsetispeed(&options, baudrate);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag |= PARENB;
  options.c_cflag |= PARODD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // options.c_iflag &= ~INPCK;

  options.c_oflag &= ~OPOST;

  options.c_iflag &= ~(IXON | IXOFF | IXANY);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &options) != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("SerialIOHandler"), "Failed to set attributes:  %s",
                std::string(strerror(errno)).c_str());
    return false;
  }

  return true;
}

void SerialIOHandler::close_fd()
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  initialized_ = false;
}

bool SerialIOHandler::is_initialized() const
{
  return initialized_;
}

void SerialIOHandler::update()
{
  if (!initialized_)
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  SerialError input_err = read_serial_input_locked();
  if (input_err != SerialError::OK)
  {
    if (input_err == SerialError::READ_FAILED || input_err == SerialError::DEVICE_DISCONNECTED)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "Serial disconnected, attempting to reopen");

      close_fd();
      initialized_ = open_port(device_path_) && configure_port(baudrate_);

      if (initialized_)
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "Serial reconnected successfully");
      else
        RCLCPP_ERROR(rclcpp::get_logger("SerialIOHandler"), "Failed to reconnect serial");
    }
  }

  SerialError output_err = write_serial_output_locked();
  if (output_err != SerialError::OK)
  {
  }

  output_err = read_serial_output_locked();
}

bool SerialIOHandler::get_input(int pin, bool& value) const
{
  if (pin < 0 || pin >= static_cast<int>(input_.size()))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  value = input_[pin];
  return true;
}

bool SerialIOHandler::set_output(int pin, bool value)
{
  if (pin < 0 || pin >= static_cast<int>(output_.size()))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  output_.at(pin) = value;
  return true;
}

bool SerialIOHandler::get_input_group(uint8_t group, uint8_t& value) const
{
  if (group != 0 && group >= (input_.size() / 8))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  value = 0x00;
  for (int i = 0; i < 8; ++i)
    value |= input_[i + group * 8] << i;

  return true;
}

bool SerialIOHandler::set_output_group(uint8_t group, uint8_t value)
{
  if (group != 0 && group >= (output_.size() / 8))
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < 8; ++i)
    output_[i + group * 8] = (value >> i) & 0x01;

  return true;
}

SerialError SerialIOHandler::read_serial_input_locked()
{
  send_request(protocol_->make_get_input_request());

  constexpr int timeout_ms = 10;
  constexpr int interval_ms = 5;
  int elapsed = 0;

  while (elapsed < timeout_ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));

    std::vector<uint8_t> package;
    SerialError err = read_response(package);
    if (err == SerialError::OK)
    {
      std::vector<uint8_t> parsed_input;
      if (protocol_->parse_get_input(package, parsed_input))
      {
        for (size_t i = 0; i < input_.size(); ++i)
        {
          size_t byte_index = i / 8;
          size_t bit_index = i % 8;
          input_[i] = (parsed_input[byte_index] >> bit_index) & 0x01;
        }
        return SerialError::OK;
      }
      else
      {
        return SerialError::PARSE_FAILED;
      }
    }
    else if (err != SerialError::FOOTER_NOT_FOUND)
    {
      return err;
    }
    elapsed += interval_ms;
  }

  return SerialError::TIMEOUT;
}

SerialError SerialIOHandler::write_serial_output_locked()
{
  std::vector<uint8_t> outputs;
  uint8_t state = 0x00;

  for (size_t i = 0; i < output_.size(); ++i)
  {
    state |= static_cast<uint8_t>(output_[i]) << (i % 8);

    if ((i + 1) % 8 == 0)
    {
      outputs.push_back(state);
      state = 0x00;
    }
  }

  send_request(protocol_->make_set_output_request(outputs));

  std::vector<uint8_t> package;
  constexpr int timeout_ms = 10;
  constexpr int interval_ms = 5;
  int elapsed = 0;

  while (elapsed < timeout_ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    SerialError err = read_response(package);

    if (err == SerialError::OK)
    {
      if (protocol_->parse_set_output(package))
      {
        return SerialError::OK;
      }
      else
      {
        return SerialError::PARSE_FAILED;
      }
    }
    else if (err != SerialError::FOOTER_NOT_FOUND)
    {
      return err;
    }
    elapsed += interval_ms;
  }

  return SerialError::TIMEOUT;
}

SerialError SerialIOHandler::read_serial_output_locked()
{
  // get_output_request();
  send_request(protocol_->make_get_output_request());

  constexpr int timeout_ms = 10;
  constexpr int interval_ms = 5;
  int elapsed = 0;

  while (elapsed < timeout_ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));

    std::vector<uint8_t> package;
    SerialError err = read_response(package);
    if (err == SerialError::OK)
    {
      std::vector<uint8_t> parsed_output;
      if (protocol_->parse_get_output(package, parsed_output))
      {
        for (size_t i = 0; i < output_state_.size(); ++i)
        {
          size_t byte_index = i / 8;
          size_t bit_index = i % 8;
          output_state_[i] = (parsed_output[byte_index] >> bit_index) & 0x01;
        }
        return SerialError::OK;
      }
      else
      {
        return SerialError::PARSE_FAILED;
      }
    }
    else if (err != SerialError::FOOTER_NOT_FOUND)
    {
      return err;
    }
    elapsed += interval_ms;
  }

  return SerialError::TIMEOUT;
}

SerialError SerialIOHandler::send_request(const std::vector<uint8_t>& package)
{
  ssize_t bytes_written = write(fd_, package.data(), package.size());

  if (bytes_written == -1)
  {
    switch (errno)
    {
      case EAGAIN:
#if EAGAIN != EWOULDBLOCK
      case EWOULDBLOCK:
#endif
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "write() timeout or would block");
        return SerialError::TIMEOUT;
      case ENODEV:
      case EIO:
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "device disconnected or I/O error");
        return SerialError::DEVICE_DISCONNECTED;
      default:
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "write() failed: %s", strerror(errno));
        return SerialError::WRITE_FAILED;
    }
  }

  return SerialError::OK;
}

SerialError SerialIOHandler::read_response(std::vector<uint8_t>& package)
{
  uint8_t temp_buffer[256];
  ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));

  if (bytes_read == -1)
  {
    switch (errno)
    {
      case EAGAIN:
#if EAGAIN != EWOULDBLOCK
      case EWOULDBLOCK:
#endif
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "read() timeout or would block");
        return SerialError::TIMEOUT;
      case ENODEV:
      case EIO:
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "device disconnected or I/O error");
        return SerialError::DEVICE_DISCONNECTED;
      default:
        RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "read() failed: %s", strerror(errno));
        return SerialError::READ_FAILED;
    }
  }

  if (bytes_read == 0)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("SerialIOHandler"), "read() returned 0 bytes");
    return SerialError::READ_FAILED;
  }

  receive_buffer_.insert(receive_buffer_.end(), temp_buffer, temp_buffer + bytes_read);

  auto start_it = std::find(receive_buffer_.begin(), receive_buffer_.end(), 0xFA);
  if (start_it == receive_buffer_.end())
  {
    receive_buffer_.clear();
    return SerialError::HEADER_NOT_FOUND;
  }

  auto end_it = std::find(start_it, receive_buffer_.end(), 0xFE);
  if (end_it == receive_buffer_.end())
  {
    return SerialError::FOOTER_NOT_FOUND;
  }

  package.assign(start_it, end_it + 1);
  receive_buffer_.erase(receive_buffer_.begin(), end_it + 1);

  return SerialError::OK;
}
