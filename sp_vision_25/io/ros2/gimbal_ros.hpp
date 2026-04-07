#ifndef IO__GIMBAL_ROS_HPP
#define IO__GIMBAL_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <example_interfaces/msg/u_int8.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "pb_rm_interfaces/msg/gimbal.hpp"

#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include <tuple>
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct GimbalState {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yaw_vel = 0.0f;
  float pitch_vel = 0.0f;
  float bullet_speed = 19.6f; // 默认值 [cite: 1]
  uint16_t bullet_count = 0;
};

class GimbalROS : public rclcpp::Node
{
public:
  GimbalROS();
  ~GimbalROS();

  int mode() const { return 1; } // 固定自瞄模式 [cite: 1]
  GimbalState state() const;

  void start_spin();   // 新增：二阶段启动
  void stop_spin();    // 新增：安全停止

  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 发送函数：支持开火指令和云台指令双发布
  void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, 
            float pitch, float pitch_vel, float pitch_acc);

    // 广播目标位置：camera系 & gimbal系（单位：米）
  void publish_target_xyz(
    const Eigen::Vector3d & xyz_camera,
    const Eigen::Vector3d & xyz_gimbal,
    const rclcpp::Time & stamp);

private:
  // 回调函数
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  
  // 发布者
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;

  // 线程与并发管理
  mutable std::mutex mutex_;
  GimbalState current_state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> queue_{1000};
  
  // 使用多线程执行器防止阻塞
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::thread spin_thread_;
  std::atomic_bool spinning_{false};
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Topics for nav usage
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_cam_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_gimbal_pub_;

};
} 

#endif