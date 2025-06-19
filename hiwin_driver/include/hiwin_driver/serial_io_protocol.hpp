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

#ifndef HIWIN_DRIVER_SERIAL_IO_PROTOCOL_HPP_
#define HIWIN_DRIVER_SERIAL_IO_PROTOCOL_HPP_

#include <vector>
#include <stdint.h>
#include <memory>

class ISerialProtocol
{
public:
  ISerialProtocol(int input_byte_size, int output_byte_size)
    : input_byte_size_(input_byte_size), output_byte_size_(output_byte_size)
  {
  }

  virtual ~ISerialProtocol() = default;

  virtual std::vector<uint8_t> make_get_input_request() const = 0;
  virtual bool parse_get_input(const std::vector<uint8_t>& package, std::vector<uint8_t>& input) const = 0;

  virtual std::vector<uint8_t> make_set_output_request(const std::vector<uint8_t>& output) const = 0;
  virtual bool parse_set_output(const std::vector<uint8_t>& package) const = 0;

  virtual std::vector<uint8_t> make_get_output_request() const = 0;
  virtual bool parse_get_output(const std::vector<uint8_t>& package, std::vector<uint8_t>& output) const = 0;

  int input_data_byte_length()
  {
    return input_byte_size_;
  }

  int output_data_byte_length()
  {
    return output_byte_size_;
  }

private:
  std::shared_ptr<ISerialProtocol> protocol_;
  int input_byte_size_;
  int output_byte_size_;
};

class DigitalIOProtocol : public ISerialProtocol
{
public:
  DigitalIOProtocol(int in_size, int out_size) : ISerialProtocol(in_size, out_size)
  {
  }

  std::vector<uint8_t> make_get_input_request() const override;
  bool parse_get_input(const std::vector<uint8_t>& package, std::vector<uint8_t>& input) const override;

  std::vector<uint8_t> make_set_output_request(const std::vector<uint8_t>& output) const override;
  bool parse_set_output(const std::vector<uint8_t>& package) const override;

  std::vector<uint8_t> make_get_output_request() const override;
  bool parse_get_output(const std::vector<uint8_t>& package, std::vector<uint8_t>& output) const override;
};

class RobotIOProtocol : public ISerialProtocol
{
public:
  RobotIOProtocol(int in_size, int out_size) : ISerialProtocol(in_size, out_size)
  {
  }

  std::vector<uint8_t> make_get_input_request() const override;
  bool parse_get_input(const std::vector<uint8_t>& package, std::vector<uint8_t>& input) const override;

  std::vector<uint8_t> make_set_output_request(const std::vector<uint8_t>& output) const override;
  bool parse_set_output(const std::vector<uint8_t>& package) const override;

  std::vector<uint8_t> make_get_output_request() const override;
  bool parse_get_output(const std::vector<uint8_t>& package, std::vector<uint8_t>& output) const override;
};

#endif  // HIWIN_DRIVER_SERIAL_IO_PROTOCOL_HPP_