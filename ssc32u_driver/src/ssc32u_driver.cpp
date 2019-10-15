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
: rclcpp_lifecycle::LifecycleNode("ssc32u", options)
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud", 9600);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SSC32UDriver::on_configure(const rclcpp_lifecycle::State &)
{
  std::string port;
  int baud;

  this->get_parameter("port", port);
  this->get_parameter("baud", baud);

  if (!this->ssc32_.open_port(port.c_str(), baud)) {
    RCLCPP_ERROR(this->get_logger(), "Unable to initialize the SSC32");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SSC32UDriver::on_activate(const rclcpp_lifecycle::State &)
{
  this->servo_command_sub_ = this->create_subscription<ssc32u_msgs::msg::ServoCommandGroup>(
    "servo_cmd", 1, std::bind(&SSC32UDriver::command_received, this, _1));

  this->discrete_output_sub_ = this->create_subscription<ssc32u_msgs::msg::DiscreteOutput>(
    "discrete_output", 1, std::bind(&SSC32UDriver::discrete_output, this, _1));

  this->query_pw_srv_ = this->create_service<ssc32u_msgs::srv::QueryPulseWidth>(
    "query_pulse_width", std::bind(&SSC32UDriver::query_pulse_width, this, _1, _2, _3));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SSC32UDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SSC32UDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SSC32UDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  this->ssc32_.close_port();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SSC32UDriver::command_received(const ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg)
{
  for (auto &command : msg->commands) {
    ssc32u_driver::SSC32::ServoCommand cmd;
    cmd.ch = command.channel;
    cmd.pw = command.pw;

    this->ssc32_.move_servo(cmd);
  }
}

void SSC32UDriver::discrete_output(const ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg)
{
  this->ssc32_.discrete_output(msg->channel, msg->output ? SSC32::High : SSC32::Low);
}

void SSC32UDriver::query_pulse_width(const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Request> request,
  std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Response> response)
{
  for (auto & channel : request->channels) {
    int pw = this->ssc32_.query_pulse_width(channel);

    response->pulse_width.push_back(pw);
  }
}

}  // namespace ssc32u_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ssc32u_driver::SSC32UDriver)
