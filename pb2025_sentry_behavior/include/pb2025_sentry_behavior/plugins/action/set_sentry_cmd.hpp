#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_CMD_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_CMD_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "pb_rm_interfaces/msg/sentry_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SetSentryCmdAction : public BT::RosTopicPubNode<pb_rm_interfaces::msg::SentryCmd>
{
public:
  SetSentryCmdAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(pb_rm_interfaces::msg::SentryCmd & msg) override;

private:
  rclcpp::Logger logger() {return node_->get_logger();}
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_CMD_HPP_