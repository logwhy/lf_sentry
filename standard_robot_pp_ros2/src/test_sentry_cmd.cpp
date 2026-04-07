#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "pb_rm_interfaces/msg/sentry_cmd.hpp"

using namespace std::chrono_literals;

class TestSentryCmdNode : public rclcpp::Node
{
public:
  TestSentryCmdNode() : Node("test_sentry_cmd_node")
  {
    pub_ = this->create_publisher<pb_rm_interfaces::msg::SentryCmd>("/sentry_cmd", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestSentryCmdNode::onTimer, this));
  }

private:
  void onTimer()
  {
    pb_rm_interfaces::msg::SentryCmd msg;

    // 你可以一次只测一个字段，最容易定位
    msg.confirm_respawn = false;
    msg.confirm_pay_respawn = false;
    msg.projectile_allowance_to_exchange = 50;
    msg.remote_projectile_request_count = 1;
    msg.remote_hp_request_count = 0;
    msg.posture_command = 2;   // ATTACK
    msg.activate_rune = false;

    pub_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "publish /sentry_cmd: respawn=%d pay_respawn=%d proj=%u remote_proj=%u remote_hp=%u posture=%u rune=%d",
      msg.confirm_respawn,
      msg.confirm_pay_respawn,
      msg.projectile_allowance_to_exchange,
      msg.remote_projectile_request_count,
      msg.remote_hp_request_count,
      msg.posture_command,
      msg.activate_rune);
  }

  rclcpp::Publisher<pb_rm_interfaces::msg::SentryCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSentryCmdNode>());
  rclcpp::shutdown();
  return 0;
}
