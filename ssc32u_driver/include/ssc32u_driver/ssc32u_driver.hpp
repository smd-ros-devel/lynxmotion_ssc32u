#ifndef SSC32U_DRIVER__SSC32U_DRIVER_HPP_
#define SSC32U_DRIVER__SSC32U_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ssc32.hpp"
#include "ssc32u_msgs/msg/servo_command_group.hpp"
#include "ssc32u_msgs/msg/discrete_output.hpp"
#include "ssc32u_msgs/srv/query_pulse_width.hpp"

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

  void command_received(const ssc32u_msgs::msg::ServoCommandGroup::SharedPtr msg);
  void discrete_output(const ssc32u_msgs::msg::DiscreteOutput::SharedPtr msg);

  void query_pulse_width(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Request> request,
    std::shared_ptr<ssc32u_msgs::srv::QueryPulseWidth::Response> response);
};

}  // namespace ssc32u_driver

#endif  // SSC32U_DRIVER__SSC32U_DRIVER_HPP_
