#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(const rclcpp::NodeOptions & options)
  : Node("publisher_node", options)
  {
    declare_parameter<std::string>("topic", "topic");
    topic_ = get_parameter("topic").as_string();
    publisher_ = this->create_publisher<std_msgs::msg::Header>(topic_, 10);

    declare_parameter<double>("publish_frequency", 10.0);
    const auto publish_frequency = get_parameter("publish_frequency").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency),
      std::bind(&PublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    msg_.stamp = rclcpp::Clock().now();
    msg_.frame_id = "frame_" + std::to_string(count_++) + " from " + topic_;
    RCLCPP_INFO(this->get_logger(), "Publishing '%s'", msg_.frame_id.c_str());
    publisher_->publish(msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  std::string topic_;
  size_t count_ = 0;
  std_msgs::msg::Header msg_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
