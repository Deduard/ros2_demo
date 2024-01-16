#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/header.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

class PublisherLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  PublisherLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("lifecycle_publisher_node", options)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    declare_parameter<std::string>("topic", "topic");
    topic_ = get_parameter("topic").as_string();
    publisher_ = this->create_publisher<std_msgs::msg::Header>(topic_, 10);

    declare_parameter<double>("publish_frequency", 10.0);
    const auto publish_frequency = get_parameter("publish_frequency").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency),
      std::bind(&PublisherLifecycleNode::timer_callback, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      msg_.stamp = rclcpp::Clock().now();
      msg_.frame_id = "frame_" + std::to_string(count_++) + " from " + topic_;
      RCLCPP_INFO(this->get_logger(), "Publishing '%s'", msg_.frame_id.c_str());
      publisher_->publish(msg_);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Header>::SharedPtr publisher_;
  std::string topic_;
  size_t count_ = 0;
  std_msgs::msg::Header msg_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PublisherLifecycleNode)
