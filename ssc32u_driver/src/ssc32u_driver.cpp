// BSD 3-Clause License
// 
// Copyright (c) 2019, Matt Richard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ssc32u_driver/ssc32u_driver.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ssc32u_driver
{

SSC32UDriver::SSC32UDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("ssc32u", options)
{
  declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<int>("baud", 115200); // TODO: Switch back to 9600 as default, but recommend using 115200
  declare_parameter<bool>("publish_pulse_width", false);
  declare_parameter<int>("publish_rate", 10); // TODO: Document that you shouldn't go higher than 10hz
  declare_parameter<int>("channel_limit", 16);

  std::string port;
  int baud;

  get_parameter("port", port);
  get_parameter("baud", baud);
  get_parameter("publish_pulse_width", publish_pulse_width_);
  get_parameter("publish_rate", publish_rate_);
  get_parameter("channel_limit", channel_limit_);

  if (!ssc32_.open_port(port.c_str(), baud)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize the SSC32");
  }

  if (publish_pulse_width_) {
    pw_pub_ = create_publisher<ssc32u_msgs::msg::PulseWidths>("pulse_widths", 10);
  }

  servo_command_sub_ = create_subscription<ssc32u_msgs::msg::ServoCommandGroup>(
    "servo_cmd", 1, std::bind(&SSC32UDriver::command_received, this, _1));

  discrete_output_sub_ = create_subscription<ssc32u_msgs::msg::DiscreteOutput>(
    "discrete_output", 1, std::bind(&SSC32UDriver::discrete_output, this, _1));

  query_pw_srv_ = create_service<ssc32u_msgs::srv::QueryPulseWidth>(
    "query_pulse_width", std::bind(&SSC32UDriver::query_pulse_width, this, _1, _2, _3));

  if (publish_pulse_width_ && publish_rate_ > 0) {
    pw_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_), std::bind(&SSC32UDriver::publish_pulse_widths, this));
  }
}

SSC32UDriver::~SSC32UDriver()
{
  ssc32_.close_port();
}

void SSC32UDriver::command_received(const ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg)
{
  for (auto &command : msg->commands) {
    ssc32u_driver::SSC32::ServoCommand cmd;
    cmd.ch = command.channel;
    cmd.pw = command.pw;

    ssc32_.move_servo(cmd);
  }
}

void SSC32UDriver::discrete_output(const ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg)
{
  ssc32_.discrete_output(msg->channel, msg->output ? SSC32::High : SSC32::Low);
}

void SSC32UDriver::query_pulse_width(const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Request> request,
  std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Response> response)
{
  for (auto & channel : request->channels) {
    int pw = ssc32_.query_pulse_width(channel);

    response->pulse_width.push_back(pw);
  }
}

void SSC32UDriver::publish_pulse_widths()
{
  auto msg = std::make_unique<ssc32u_msgs::msg::PulseWidths>();

  for (int i = 0; i < channel_limit_; i++) {
    int pw = ssc32_.query_pulse_width(i);

    ssc32u_msgs::msg::PulseWidth ch;
    ch.channel = i;
    ch.pw = pw;

    msg->channels.push_back(ch);
  }

  pw_pub_->publish(std::move(msg));
}

}  // namespace ssc32u_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ssc32u_driver::SSC32UDriver)
