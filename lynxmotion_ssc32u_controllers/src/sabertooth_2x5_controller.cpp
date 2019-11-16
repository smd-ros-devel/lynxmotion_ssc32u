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

#include "lynxmotion_ssc32u_controllers/sabertooth_2x5_controller.hpp"

#include <utility>
#include <algorithm>

using std::placeholders::_1;

namespace lynxmotion_ssc32u_controllers
{

Sabertooth2x5Controller::Sabertooth2x5Controller(const rclcpp::NodeOptions & options)
: Node("sabertooth_2x5_controller", options)
{
  process_parameters();
  setup_subscriptions();
  setup_publishers();
}

int Sabertooth2x5Controller::clamp_pulse_width(int pulse_width)
{
  if (pulse_width < 500) {
		return 500;
  }

  if (pulse_width > 2500) {
    return 2500;
  }

  return pulse_width;
}

void Sabertooth2x5Controller::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto command_msg = std::make_unique<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>();

	double ch1_vel = (2.0 * msg->linear.x - msg->angular.z * wheel_base_) / wheel_diam_;
	double ch2_vel = (2.0 * msg->linear.x + msg->angular.z * wheel_base_) / wheel_diam_;

  ch1_vel = std::min(ch1_vel / max_vel_, 1.0);
  ch2_vel = std::min(ch2_vel / max_vel_, 1.0);
  
  lynxmotion_ssc32u_msgs::msg::ServoCommand ch1_command;
  ch1_command.channel = ch1_channel_;
  ch1_command.pw = clamp_pulse_width(1000 * ch1_vel + 1500);

  lynxmotion_ssc32u_msgs::msg::ServoCommand ch2_command;
  ch2_command.channel = ch2_channel_;
  ch2_command.pw = clamp_pulse_width(1000 * ch2_vel + 1500);

  command_msg->commands.push_back(ch1_command);
  command_msg->commands.push_back(ch2_command);

  servo_command_pub_->publish(std::move(command_msg));
}

void Sabertooth2x5Controller::pulse_widths_callback(const lynxmotion_ssc32u_msgs::msg::PulseWidths::SharedPtr)
{
  
}

void Sabertooth2x5Controller::process_parameters()
{
  declare_parameter<int>("ch1_channel", 0);
  declare_parameter<int>("ch2_channel", 1);
  declare_parameter<double>("max_vel", 1.0);
  declare_parameter<bool>("independent_control", true);
  declare_parameter<double>("wheel_diam", 1.0);
  declare_parameter<double>("wheel_base", 1.0);

  get_parameter("ch1_channel", ch1_channel_);
  get_parameter("ch2_channel", ch2_channel_);
  get_parameter("max_vel", max_vel_);
  get_parameter("independent_control", independent_control_);
  get_parameter<double>("wheel_diam", wheel_diam_);
  get_parameter<double>("wheel_base", wheel_base_);
}

void Sabertooth2x5Controller::setup_subscriptions()
{
  pulse_width_sub_ = create_subscription<lynxmotion_ssc32u_msgs::msg::PulseWidths>(
    "pulse_widths",
    1,
    std::bind(&Sabertooth2x5Controller::pulse_widths_callback, this, _1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    1,
    std::bind(&Sabertooth2x5Controller::cmd_vel_callback, this, _1));
}

void Sabertooth2x5Controller::setup_publishers()
{
  servo_command_pub_ = create_publisher<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>(
    "servo_cmd",
    rclcpp::QoS(rclcpp::KeepLast(1)));
}

}  // namespace lynxmotion_ssc32u_controllers

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lynxmotion_ssc32u_controllers::Sabertooth2x5Controller)
