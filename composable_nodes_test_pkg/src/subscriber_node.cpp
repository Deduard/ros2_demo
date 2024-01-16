#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class SubscriberNode : public rclcpp::Node
{
  typedef message_filters::sync_policies::ApproximateTime<std_msgs::msg::Header,
      std_msgs::msg::Header, std_msgs::msg::Header> MySyncPolicy;

public:
  SubscriberNode(const rclcpp::NodeOptions & options)
  : Node("subscriber_node", options)
  {
    sub1_ = std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(this, "topic1");
    sub2_ = std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(this, "topic2");
    sub3_ = std::make_shared<message_filters::Subscriber<std_msgs::msg::Header>>(this, "topic3");

    typedef message_filters::sync_policies::ApproximateTime<std_msgs::msg::Header,
        std_msgs::msg::Header, std_msgs::msg::Header> MySyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(
        30), *sub1_, *sub2_, *sub3_);
    sync_->registerCallback(
      std::bind(
        &SubscriberNode::callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
  }

private:
  void callback(
    const std_msgs::msg::Header::ConstSharedPtr & msg1,
    const std_msgs::msg::Header::ConstSharedPtr & msg2,
    const std_msgs::msg::Header::ConstSharedPtr & msg3)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received:\n - '%s'\n - '%s'\n - '%s'",
      msg1->frame_id.c_str(), msg2->frame_id.c_str(), msg3->frame_id.c_str());
  }

  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub1_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub2_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Header>> sub3_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberNode)
