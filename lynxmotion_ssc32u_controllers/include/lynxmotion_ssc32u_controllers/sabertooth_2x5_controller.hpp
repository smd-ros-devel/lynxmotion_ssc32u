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

#ifndef LYNXMOTION_SSC32U_CONTROLLERS__SERVO_CONTROLLER_HPP_
#define LYNXMOTION_SSC32U_CONTROLLERS__SERVO_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lynxmotion_ssc32u_msgs/msg/servo_command_group.hpp"
#include "lynxmotion_ssc32u_msgs/msg/pulse_widths.hpp"

namespace lynxmotion_ssc32u_controllers
{

class Sabertooth2x5Controller : public rclcpp::Node
{
public:
  explicit Sabertooth2x5Controller(const rclcpp::NodeOptions & options);

  int clamp_pulse_width(int pulse_width);

private:
  int ch1_channel_;
  int ch2_channel_;
  double max_vel_;
  bool independent_control_;
  double wheel_diam_;
  double wheel_base_;

  rclcpp::Subscription<lynxmotion_ssc32u_msgs::msg::PulseWidths>::SharedPtr pulse_width_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>::SharedPtr servo_command_pub_;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pulse_widths_callback(const lynxmotion_ssc32u_msgs::msg::PulseWidths::SharedPtr msg);

  void process_parameters();
  void setup_subscriptions();
  void setup_publishers();
};

}  // namespace lynxmotion_ssc32u_controllers

#endif  // LYNXMOTION_SSC32U_CONTROLLERS__SERVO_CONTROLLER_HPP_
