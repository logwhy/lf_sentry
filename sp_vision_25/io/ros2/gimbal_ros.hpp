#ifndef IO__GIMBAL_ROS_HPP
#define IO__GIMBAL_ROS_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include <example_interfaces/msg/float32.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pb_rm_interfaces/msg/gimbal.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct GimbalState
{
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yaw_vel = 0.0f;
  float pitch_vel = 0.0f;
  float bullet_speed = 19.6f;
  uint16_t bullet_count = 0;
};

class GimbalROS : public rclcpp::Node
{
public:
  GimbalROS();
  ~GimbalROS();

  int mode() const { return 1; }
  GimbalState state() const;

  void start_spin();
  void stop_spin();
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc);

  void publish_target_xyz(
    const Eigen::Vector3d & xyz_camera, const Eigen::Vector3d & xyz_gimbal,
    const rclcpp::Time & stamp);

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void bullet_speed_callback(const example_interfaces::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr bullet_speed_sub_;

  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;

  mutable std::mutex mutex_;
  GimbalState current_state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::thread spin_thread_;
  std::atomic_bool spinning_{false};
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_cam_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_gimbal_pub_;
};
}  // namespace io

#endif  // IO__GIMBAL_ROS_HPP
