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

#include "rclcpp/rclcpp.hpp"

#include "hiwin_driver/serial_io_protocol.hpp"

/*
 *
 */
std::vector<uint8_t> DigitalIOProtocol::make_get_input_request() const
{
  std::vector<uint8_t> package = { 0xFA, 0x56, 0x0F, 0xC1, 0x00, 0xFE };
  package[4] = package[2] + package[3];
  return package;
}

bool DigitalIOProtocol::parse_get_input(const std::vector<uint8_t>& package, std::vector<uint8_t>& input) const
{
  /*
   *  byte[0] | byte[1] | byte[2] | byte[3] | byte[4] | byte[5] | byte[6] | byte[7]
   *   0xFA   |  0x56   |  Data1  |  Data2  |  Data3  |  Data4  |   SUM   |  0xFE
   */

  input.push_back(package[2]);
  input.push_back(package[3]);
  input.push_back(package[4]);
  input.push_back(package[5]);

  return true;
}

std::vector<uint8_t> DigitalIOProtocol::make_set_output_request(const std::vector<uint8_t>& output) const
{
  if (output.size() != 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DigitalIO"), "Invalid Output length");
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> package = { 0xFA, 0xFD, output.at(0), output.at(1), output.at(2), output.at(3), 0, 0xFE };
  package[6] = package[2] + package[3] + package[4] + package[5];
  return package;
}

bool DigitalIOProtocol::parse_set_output(const std::vector<uint8_t>& package) const
{
  /*
   * [0xFA][0xFD][0x0F][0x55][0x64][0xFE]
   * Comfirm package :
   *   [0xFA][0xFD][0x0F][0x55][0x64][0xFE]
   * Error package :
   *  [0xFA][0xFD][0x00][0x55][0x55][0xFE]
   *  [0xFA][0xFD][0x00][0xAA][0xAA][0xFE]
   */

  if (package[2] != 0x0F && package[3] != 0x55)
    return false;

  return true;
}

std::vector<uint8_t> DigitalIOProtocol::make_get_output_request() const
{
  std::vector<uint8_t> package = { 0xFA, 0x56, 0x0F, 0xC0, 0x00, 0xFE };
  package[4] = package[2] + package[3];
  return package;
}

bool DigitalIOProtocol::parse_get_output(const std::vector<uint8_t>& package, std::vector<uint8_t>& output) const
{
  /*
   *  byte[0] | byte[1] | byte[2] | byte[3] | byte[4] | byte[5] | byte[6] | byte[7]
   *   0xFA   |  0xFD   |  Data1  |  Data2  |  Data3  |  Data4  |   SUM   |  0xFE
   */

  output.push_back(package[2]);
  output.push_back(package[3]);
  output.push_back(package[4]);
  output.push_back(package[5]);

  return true;
}

/*
 *
 */
std::vector<uint8_t> RobotIOProtocol::make_get_input_request() const
{
  std::vector<uint8_t> package = { 0xFA, 0x56, 0x0F, 0xC1, 0x00, 0xFE };
  package[4] = package[2] + package[3];
  return package;
}

bool RobotIOProtocol::parse_get_input(const std::vector<uint8_t>& package, std::vector<uint8_t>& input) const
{
  /*
   * [0xFA][0xFD][Data1][Data2][checksum][0xFE]
   * Data1 = [0, 1, X, X, RI4, RI3, RI2, RI1]
   * Data2 = [1, 0, X, X, RI8, RI7, RI6, RI5]
   * checksum = Data1 + Data2
   */

  input.push_back(package[2]);
  input.push_back(package[3]);

  return true;
}

std::vector<uint8_t> RobotIOProtocol::make_set_output_request(const std::vector<uint8_t>& output) const
{
  if (output.size() != 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotIO"), "Invalid Output length");
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> package = { 0xFA, 0xFD, output.at(0), output.at(1), output.at(2), output.at(3), 0, 0xFE };
  package[6] = package[2] + package[3] + package[4] + package[5];
  return package;
}

bool RobotIOProtocol::parse_set_output(const std::vector<uint8_t>& package) const
{
  /*
   * [0xFA][0xFD][0x0F][0x55][0x64][0xFE]
   * Comfirm package :
   *   [0xFA][0xFD][0x0F][0x55][0x64][0xFE]
   * Error package :
   *  [0xFA][0xFD][0x00][0x55][0x55][0xFE]
   *  [0xFA][0xFD][0x00][0xAA][0xAA][0xFE]
   */

  if (package[2] != 0x0F && package[3] != 0x55)
    return false;

  return true;
}

std::vector<uint8_t> RobotIOProtocol::make_get_output_request() const
{
  std::vector<uint8_t> package = { 0xFA, 0x56, 0x0F, 0xC0, 0x00, 0xFE };
  package[4] = package[2] + package[3];
  return package;
}

bool RobotIOProtocol::parse_get_output(const std::vector<uint8_t>& package, std::vector<uint8_t>& output) const
{
  /*
   * [0xFA][0XFD][Data1][Data2][Data3][checksum][0xFE]
   * Data1 = [1, 1, VO_EN3, VO_EN2, VO_EN1, VO3, VO2, VO1]
   * Data2 = [0, 1, X, X, RO4, RO3, RO2, RO1]
   * Data3 = [1, 0, X, X, RO8, RO7, RO6, RO5]
   * checksum = Data1 + Data2 + Data3
   */

  output.push_back(package[2]);
  output.push_back(package[3]);
  output.push_back(package[4]);

  return true;
}