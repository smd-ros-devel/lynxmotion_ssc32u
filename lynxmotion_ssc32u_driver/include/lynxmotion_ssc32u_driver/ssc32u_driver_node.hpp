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

#ifndef LYNXMOTION_SSC32U_DRIVER__LYNXMOTION_SSC32U_DRIVER_HPP_
#define LYNXMOTION_SSC32U_DRIVER__LYNXMOTION_SSC32U_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ssc32.hpp"
#include "lynxmotion_ssc32u_msgs/msg/servo_command_group.hpp"
#include "lynxmotion_ssc32u_msgs/msg/discrete_output.hpp"
#include "lynxmotion_ssc32u_msgs/srv/query_pulse_width.hpp"
#include "lynxmotion_ssc32u_msgs/msg/pulse_width.hpp"
#include "lynxmotion_ssc32u_msgs/msg/pulse_widths.hpp"

namespace lynxmotion_ssc32u_driver
{

class SSC32UDriverNode : public rclcpp::Node
{
public:
  explicit SSC32UDriverNode(const rclcpp::NodeOptions & options);
  ~SSC32UDriverNode();

private:
  SSC32 ssc32_;
  rclcpp::Subscription<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>::SharedPtr servo_command_sub_;
  rclcpp::Subscription<lynxmotion_ssc32u_msgs::msg::DiscreteOutput>::SharedPtr discrete_output_sub_;
  rclcpp::Service<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth>::SharedPtr query_pw_srv_;

  rclcpp::Publisher<lynxmotion_ssc32u_msgs::msg::PulseWidths>::SharedPtr pw_pub_;
  std::shared_ptr<rclcpp::TimerBase> pw_timer_;

  bool publish_pulse_width_;
  int publish_rate_;
  int channel_limit_;

  void command_received(const lynxmotion_ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg);
  void discrete_output(const lynxmotion_ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg);
  void publish_pulse_widths();

  void query_pulse_width(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth::Request> request,
    std::shared_ptr<lynxmotion_ssc32u_msgs::srv::QueryPulseWidth::Response> response);
};

}  // namespace lynxmotion_ssc32u_driver

#endif  // LYNXMOTION_SSC32U_DRIVER__LYNXMOTION_SSC32U_DRIVER_HPP_
