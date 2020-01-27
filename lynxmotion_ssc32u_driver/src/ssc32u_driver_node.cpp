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

#include "lynxmotion_ssc32u_driver/ssc32u_driver_node.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace lynxmotion_ssc32u_driver
{

SSC32UDriverNode::SSC32UDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ssc32u_driver", options)
{
  process_parameters();
  setup_subscriptions();
  setup_publishers();
  setup_clients();

  init();
}

SSC32UDriverNode::~SSC32UDriverNode()
{
  ssc32_.close_port();
}

void SSC32UDriverNode::init()
{
  if (!ssc32_.open_port(port_.c_str(), baud_)) {
    RCLCPP_ERROR(get_logger(), "Unable to initialize the SSC32");
  }
}

void SSC32UDriverNode::command_received(const lynxmotion_ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg)
{
  std::vector<lynxmotion_ssc32u_driver::SSC32::ServoCommand> device_commands;

  for (auto &command : msg->commands) {
    lynxmotion_ssc32u_driver::SSC32::ServoCommand cmd;
    cmd.ch = command.channel;
    cmd.pw = command.pw;

    device_commands.push_back(cmd);
  }

  ssc32_.move_servo(&device_commands[0], device_commands.size());
}

void SSC32UDriverNode::discrete_output(const lynxmotion_ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg)
{
  ssc32_.discrete_output(msg->channel, msg->output ? SSC32::High : SSC32::Low);
}

void SSC32UDriverNode::query_pulse_width(const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth::Request> request,
  std::shared_ptr<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth::Response> response)
{
  for (auto & channel : request->channels) {
    int pw = ssc32_.query_pulse_width(channel);

    response->pulse_width.push_back(pw);
  }
}

void SSC32UDriverNode::publish_pulse_widths()
{
  auto msg = std::make_unique<lynxmotion_ssc32u_msgs::msg::PulseWidths>();

  for (int i = 0; i < channel_limit_; i++) {
    int pw = ssc32_.query_pulse_width(i);

    lynxmotion_ssc32u_msgs::msg::PulseWidth ch;
    ch.channel = i;
    ch.pw = pw;

    msg->channels.push_back(ch);
  }

  pw_pub_->publish(std::move(msg));
}

void SSC32UDriverNode::process_parameters()
{
  declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<int>("baud", 9600);
  declare_parameter<bool>("publish_pulse_width", true);
  declare_parameter<int>("publish_rate", 5); // TODO: Document that you shouldn't go higher than 10hz
  declare_parameter<int>("channel_limit", 16);

  get_parameter("port", port_);
  get_parameter("baud", baud_);
  get_parameter("publish_pulse_width", publish_pulse_width_);
  get_parameter("publish_rate", publish_rate_);
  get_parameter("channel_limit", channel_limit_);
}

void SSC32UDriverNode::setup_subscriptions()
{
  servo_command_sub_ = create_subscription<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>(
    "servo_cmd", 1, std::bind(&SSC32UDriverNode::command_received, this, _1));

  discrete_output_sub_ = create_subscription<lynxmotion_ssc32u_msgs::msg::DiscreteOutput>(
    "discrete_output", 1, std::bind(&SSC32UDriverNode::discrete_output, this, _1));
}

void SSC32UDriverNode::setup_publishers()
{
  if (publish_pulse_width_) {
    pw_pub_ = create_publisher<lynxmotion_ssc32u_msgs::msg::PulseWidths>("pulse_widths", 10);

    if (publish_rate_ > 0) {
      pw_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / publish_rate_), std::bind(&SSC32UDriverNode::publish_pulse_widths, this));
    }
  }
}

void SSC32UDriverNode::setup_clients()
{
  query_pw_srv_ = create_service<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth>(
    "query_pulse_width", std::bind(&SSC32UDriverNode::query_pulse_width, this, _1, _2, _3));
}

}  // namespace lynxmotion_ssc32u_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lynxmotion_ssc32u_driver::SSC32UDriverNode)
