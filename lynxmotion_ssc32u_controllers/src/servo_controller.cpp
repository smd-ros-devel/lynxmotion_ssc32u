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

#include "lynxmotion_ssc32u_controllers/servo_controller.hpp"

#include <utility>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace lynxmotion_ssc32u_controllers
{

ServoController::ServoController(const rclcpp::NodeOptions & options = (
  rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
))
: Node("lynxmotion_ssc32u_servo_controller", options)
{
  process_parameters();
  setup_subscriptions();
  setup_publishers();
  setup_services();
}

int ServoController::clamp_pulse_width(int pulse_width)
{
  if (pulse_width < 500) {
		return 500;
  }

  if (pulse_width > 2500) {
    return 2500;
  }

  return pulse_width;
}

int ServoController::invert_pulse_width(int pulse_width)
{
  return 3000 - pulse_width;
}

void ServoController::relax_joints()
{
  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++) {
    auto msg = std::make_unique<lynxmotion_ssc32u_msgs::msg::DiscreteOutput>();
    msg->channel = it->second.channel;
    msg->output = 0; // Low

    discrete_output_pub_->publish(std::move(msg));
  }
}

void ServoController::init()
{
  if (!joints_pos_map_.empty()) {
    init_timer_->cancel();
    return;
  }

  auto command_msg = std::make_unique<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>();
  double scale = 1.0 * 2000.0 / M_PI;

  for (auto it = joints_map_.begin(); it != joints_map_.end(); it++) {
    auto joint = it->second;
    if (joint.initialize) {
      lynxmotion_ssc32u_msgs::msg::ServoCommand command;
      command.channel = joint.channel;
      command.pw = (unsigned int)(scale * (joint.default_angle - joint.offset_angle) + 1500 + 0.5);

      command.pw = clamp_pulse_width(command.pw);

      if (joint.invert) {
        command.pw = invert_pulse_width(command.pw);
      }

      RCLCPP_INFO(get_logger(), "Initializing channel %d to pulse width %d", command.channel, command.pw);

      command_msg->commands.push_back(command);
    }
  }

  if (command_msg->commands.size() > 0) {
    servo_command_pub_->publish(std::move(command_msg));
  }
}

void ServoController::pulse_widths_callback(const lynxmotion_ssc32u_msgs::msg::PulseWidths::SharedPtr msg)
{
  if (!publish_joint_states_) {
    return;
  }

  double scale = 1.0 * 2000.0 / M_PI;
  
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();

  for (unsigned int i = 0; i < msg->channels.size(); i++) {
    auto channel = msg->channels[i];

    Joint * joint = nullptr;

    // Find the joint corresponding with the current channel
    for (auto it = joints_map_.begin(); it != joints_map_.end(); it++) {
      if (it->second.channel == channel.channel) {
        joint = &(it->second);
        break;
      }
    }

    if (joint != nullptr) {
      int pw = channel.pw;
      if (pw > 0) {
        if (joint->invert) {
          pw = this->invert_pulse_width(pw);
        }

        double angle = ((double)pw - 1500.0) / scale + joint->offset_angle;

        joint_state_msg->position.push_back(angle);
        joint_state_msg->name.push_back(joint->name);

        joints_pos_map_[joint->name] = angle;
      }
    }
  }

  if (joint_state_msg->name.size() > 0) {
    this->joint_state_pub_->publish(std::move(joint_state_msg));
  }
}

void ServoController::joint_command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  auto command_msg = std::make_unique<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>();
  bool invalid = false;

  for (unsigned int i = 0; i < msg->points.size(); i++) {
		for (unsigned int j = 0; j < msg->joint_names.size() && !invalid; j++) {
      lynxmotion_ssc32u_msgs::msg::ServoCommand command;

      if (joints_map_.find(msg->joint_names[j]) != joints_map_.end()) {
				Joint joint = joints_map_[msg->joint_names[j]];

				double angle = msg->points[i].positions[j];
        double scale = 1.0 * 2000.0 / M_PI;

				// Validate the commanded position (angle)
				if (angle >= joint.min_angle && angle <= joint.max_angle) {
					command.channel = joint.channel;
					command.pw = (unsigned int)(scale * (angle - joint.offset_angle) + 1500 + 0.5);

          command.pw = clamp_pulse_width(command.pw);

					if (joint.invert) {
						command.pw = invert_pulse_width(command.pw);
          }

					if (msg->points[i].velocities.size() > j && msg->points[i].velocities[j] > 0) {
						command.speed = scale * msg->points[i].velocities[j];
          }
				} else { // invalid angle given
					invalid = true;
					RCLCPP_ERROR(get_logger(), "The given position [%f] for joint [%s] is invalid", angle, joint.name.c_str());
				}
			} else {
				invalid = true;
				RCLCPP_ERROR(get_logger(), "Joint [%s] does not exist", msg->joint_names[i].c_str());
			}

      command_msg->commands.push_back(command);
    }
  }

  if (!invalid) {
    servo_command_pub_->publish(std::move(command_msg));
  }
}

void ServoController::relax_joints_callback(const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  relax_joints();
}

void ServoController::process_parameters()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto joint_parameters = parameters_client->list_parameters({"joints"}, 3);

  if (joint_parameters.names.size() == 0) {
    RCLCPP_WARN(get_logger(), "No joints were provided");
  }

  // Extract joint names from parameter list
  std::set<std::string> joint_names_set;
  for (auto & name : joint_parameters.names) {
    int start = name.find(".");
    int end = name.find(".", start + 1);
    std::string joint_name = name.substr(start + 1, end - start - 1);

    joint_names_set.insert(joint_name);
  }

  // Extract joint properties
  for (auto & name : joint_names_set) {
    Joint joint;
    joint.name = name;

    get_parameter("joints." + name + ".channel", joint.channel);
    get_parameter("joints." + name + ".max_angle", joint.max_angle);
    get_parameter("joints." + name + ".min_angle", joint.min_angle);
    get_parameter("joints." + name + ".offset_angle", joint.offset_angle);
    get_parameter("joints." + name + ".default_angle", joint.default_angle);
    get_parameter("joints." + name + ".initialize", joint.initialize);
    get_parameter("joints." + name + ".invert", joint.invert);

    joints_map_[name] = joint;
  }

  get_parameter("publish_joint_states", publish_joint_states_);
}

void ServoController::setup_subscriptions()
{
  pulse_width_sub_ = create_subscription<lynxmotion_ssc32u_msgs::msg::PulseWidths>(
    "pulse_widths",
    1,
    std::bind(&ServoController::pulse_widths_callback, this, _1));

  joint_trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "command",
    1,
    std::bind(&ServoController::joint_command_callback, this, _1));
}

void ServoController::setup_publishers()
{
  discrete_output_pub_ = create_publisher<lynxmotion_ssc32u_msgs::msg::DiscreteOutput>(
    "discrete_output",
    rclcpp::QoS(rclcpp::KeepLast(1)));

  servo_command_pub_ = create_publisher<lynxmotion_ssc32u_msgs::msg::ServoCommandGroup>(
    "servo_cmd",
    rclcpp::QoS(rclcpp::KeepLast(1)));

  if (publish_joint_states_) {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&ServoController::init, this));
}

void ServoController::setup_services()
{
  relax_joints_srv_ = create_service<std_srvs::srv::Empty>(
    "relax_joints",
    std::bind(&ServoController::relax_joints_callback, this, _1, _2, _3));
}

}  // namespace lynxmotion_ssc32u_controllers

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lynxmotion_ssc32u_controllers::ServoController)
