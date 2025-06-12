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

#ifndef HIWIN_DRIVER_SERIAL_IO_HANDLER_HPP_
#define HIWIN_DRIVER_SERIAL_IO_HANDLER_HPP_

#include <vector>
#include <mutex>
#include <string>

#include "hiwin_driver/device_handler.hpp"
#include "hiwin_driver/serial_io_protocol.hpp"

enum class SerialError
{
  OK = 0,
  WRITE_FAILED,
  READ_FAILED,
  HEADER_NOT_FOUND,
  FOOTER_NOT_FOUND,
  PARSE_FAILED,
  TIMEOUT,
  DEVICE_DISCONNECTED,
};

class SerialIOHandler : public IDeviceHandler
{
public:
  SerialIOHandler(std::shared_ptr<ISerialProtocol> protocol);
  ~SerialIOHandler();

  bool init(const std::string& device_path, int baudrate);
  bool is_initialized() const;
  void update() override;

  bool get_input(int pin, bool& value) const;
  bool set_output(int pin, bool value);
  bool get_input_group(uint8_t group, uint8_t& value) const;
  bool set_output_group(uint8_t group, uint8_t value);

private:
  bool open_port(const std::string& device_path);
  bool configure_port(int baudrate);
  void close_fd();

  SerialError read_serial_input_locked();
  SerialError write_serial_output_locked();
  SerialError read_serial_output_locked();
  SerialError send_request(const std::vector<uint8_t>& package);
  SerialError read_response(std::vector<uint8_t>& package);

  // void get_input_request();
  // bool parse_get_input(std::vector<uint8_t>& package, std::vector<uint8_t>& input);
  // void set_output_request(std::vector<uint8_t> output);
  // bool parse_set_output(std::vector<uint8_t>& package);
  // void get_output_request();
  // bool parse_get_output(std::vector<uint8_t>& package, std::vector<uint8_t>& output);

  int fd_;
  bool initialized_;
  std::string device_path_;
  int baudrate_;
  mutable std::mutex mutex_;
  std::vector<uint8_t> receive_buffer_;
  std::vector<bool> input_;
  std::vector<bool> output_;
  std::vector<bool> output_state_;

  std::shared_ptr<ISerialProtocol> protocol_;
};

#endif  // HIWIN_DRIVER_SERIAL_IO_HANDLER_HPP_
