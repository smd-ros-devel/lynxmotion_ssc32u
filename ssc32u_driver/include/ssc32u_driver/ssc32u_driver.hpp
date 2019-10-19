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

#ifndef SSC32U_DRIVER__SSC32U_DRIVER_HPP_
#define SSC32U_DRIVER__SSC32U_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ssc32.hpp"
#include "ssc32u_msgs/msg/servo_command_group.hpp"
#include "ssc32u_msgs/msg/discrete_output.hpp"
#include "ssc32u_msgs/srv/query_pulse_width.hpp"
#include "ssc32u_msgs/msg/pulse_width.hpp"
#include "ssc32u_msgs/msg/pulse_widths.hpp"

namespace ssc32u_driver
{

class SSC32UDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SSC32UDriver(const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

private:
  SSC32 ssc32_;
  rclcpp::Subscription<ssc32u_msgs::msg::ServoCommandGroup>::SharedPtr servo_command_sub_;
  rclcpp::Subscription<ssc32u_msgs::msg::DiscreteOutput>::SharedPtr discrete_output_sub_;
  rclcpp::Service<ssc32u_msgs::srv::QueryPulseWidth>::SharedPtr query_pw_srv_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ssc32u_msgs::msg::PulseWidths>> pw_pub_;
  std::shared_ptr<rclcpp::TimerBase> pw_timer_;

  bool publish_pulse_width_;
  int publish_rate_;

  void command_received(const ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg);
  void discrete_output(const ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg);
  void publish_pulse_widths();

  void query_pulse_width(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Request> request,
    std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Response> response);
};

}  // namespace ssc32u_driver

#endif  // SSC32U_DRIVER__SSC32U_DRIVER_HPP_
