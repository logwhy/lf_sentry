#include "pb2025_sentry_behavior/plugins/action/calculate_chase_goal.hpp"

#include <cmath>
#include <mutex>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

CalculateChaseGoal::CalculateChaseGoal(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList CalculateChaseGoal::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/target_xyz_camera", "Vision target topic"),
    BT::InputPort<std::string>(
      "source_frame", "front_industrial_camera_optical_frame",
      "Real source frame used for target point"),
    BT::InputPort<std::string>("world_frame", "odom", "World frame"),
    BT::InputPort<std::string>("robot_base_frame", "chassis", "Robot base frame"),

    BT::InputPort<double>("target_timeout", 0.35, "Target timeout in seconds"),
    BT::InputPort<double>("chase_distance", 1.5, "Desired stop distance to target"),
    BT::InputPort<double>("stop_tolerance", 0.15, "Tolerance around chase distance"),

    // 相机系筛选
    BT::InputPort<double>("z_min", 0.8, "Minimum valid target depth"),
    BT::InputPort<double>("z_max", 6.0, "Maximum valid target depth"),
    BT::InputPort<double>("x_abs_max", 2.5, "Maximum abs x in camera frame"),
    BT::InputPort<double>("y_abs_max", 1.5, "Maximum abs y in camera frame"),

    // 目标点区域
    BT::InputPort<double>("world_x_min", -100.0, "World min x for target"),
    BT::InputPort<double>("world_x_max", 100.0, "World max x for target"),
    BT::InputPort<double>("world_y_min", -100.0, "World min y for target"),
    BT::InputPort<double>("world_y_max", 100.0, "World max y for target"),

    // 追击点区域
    BT::InputPort<double>("goal_x_min", -100.0, "Goal min x"),
    BT::InputPort<double>("goal_x_max", 100.0, "Goal max x"),
    BT::InputPort<double>("goal_y_min", -100.0, "Goal min y"),
    BT::InputPort<double>("goal_y_max", 100.0, "Goal max y"),

    BT::InputPort<double>("transform_tolerance", 0.2, "TF timeout"),

    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "{chase_goal}", "Chase goal")
  };
}

void CalculateChaseGoal::initOnce()
{
  if (inited_) {
    return;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError(
      std::string("CalculateChaseGoal failed to get ROS node from blackboard: ") + e.what());
  }

  getInput("topic_name", topic_name_);
  getInput("source_frame", source_frame_);
  getInput("world_frame", world_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("target_timeout", target_timeout_);
  getInput("chase_distance", chase_distance_);
  getInput("stop_tolerance", stop_tolerance_);
  getInput("z_min", z_min_);
  getInput("z_max", z_max_);
  getInput("x_abs_max", x_abs_max_);
  getInput("y_abs_max", y_abs_max_);
  getInput("world_x_min", world_x_min_);
  getInput("world_x_max", world_x_max_);
  getInput("world_y_min", world_y_min_);
  getInput("world_y_max", world_y_max_);
  getInput("goal_x_min", goal_x_min_);
  getInput("goal_x_max", goal_x_max_);
  getInput("goal_y_min", goal_y_min_);
  getInput("goal_y_max", goal_y_max_);
  getInput("transform_tolerance", transform_tolerance_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    topic_name_,
    rclcpp::SensorDataQoS(),
    std::bind(&CalculateChaseGoal::targetCallback, this, std::placeholders::_1));

  inited_ = true;

  RCLCPP_INFO(
    logger_,
    "[CalculateChaseGoal] topic=%s source_frame=%s world_frame=%s robot_base_frame=%s",
    topic_name_.c_str(), source_frame_.c_str(), world_frame_.c_str(), robot_base_frame_.c_str());
}

void CalculateChaseGoal::targetCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_target_ = *msg;
  last_recv_time_ = node_->now();
  has_target_ = true;
}

BT::NodeStatus CalculateChaseGoal::tick()
{
  initOnce();

  geometry_msgs::msg::PointStamped target_cam;
  rclcpp::Time recv_time(0, 0, node_->get_clock()->get_clock_type());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_target_) {
      RCLCPP_DEBUG(logger_, "[CalculateChaseGoal] no target received yet");
      return BT::NodeStatus::FAILURE;
    }
    target_cam = last_target_;
    recv_time = last_recv_time_;
  }

  const double dt = (node_->now() - recv_time).seconds();
  if (dt > target_timeout_) {
    RCLCPP_DEBUG(
      logger_, "[CalculateChaseGoal] target expired, dt=%.3f > %.3f", dt, target_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  // 不信消息自带 frame_id，只把它当“点容器”使用
  target_cam.header.frame_id = source_frame_;

  // /target_xyz_camera 语义：z=深度距离，x/y=左右/上下偏移
  const double cx = target_cam.point.x;
  const double cy = target_cam.point.y;
  const double cz = target_cam.point.z;

  if (cz < z_min_ || cz > z_max_) {
    RCLCPP_DEBUG(
      logger_, "[CalculateChaseGoal] reject by depth z=%.3f not in [%.3f, %.3f]",
      cz, z_min_, z_max_);
    return BT::NodeStatus::FAILURE;
  }

  if (std::abs(cx) > x_abs_max_ || std::abs(cy) > y_abs_max_) {
    RCLCPP_DEBUG(
      logger_,
      "[CalculateChaseGoal] reject by camera offset x=%.3f y=%.3f (limit %.3f %.3f)",
      cx, cy, x_abs_max_, y_abs_max_);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PointStamped target_world;
  try {
    target_world = tf_buffer_->transform(
      target_cam, world_frame_, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "[CalculateChaseGoal] TF transform failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double tx = target_world.point.x;
  const double ty = target_world.point.y;

  // 目标点区域限制
  if (tx < world_x_min_ || tx > world_x_max_ || ty < world_y_min_ || ty > world_y_max_) {
    RCLCPP_DEBUG(
      logger_,
      "[CalculateChaseGoal] reject by target area target=(%.3f, %.3f), area=[%.3f, %.3f] x [%.3f, %.3f]",
      tx, ty, world_x_min_, world_x_max_, world_y_min_, world_y_max_);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
        robot_pose, *tf_buffer_, world_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(logger_, "[CalculateChaseGoal] failed to get robot pose");
    return BT::NodeStatus::FAILURE;
  }

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  const double dx = tx - rx;
  const double dy = ty - ry;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-6) {
    RCLCPP_DEBUG(logger_, "[CalculateChaseGoal] distance too small");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = world_frame_;
  goal.header.stamp = node_->now();

  double gx = rx;
  double gy = ry;

  // 只有距离明显大于 1.5m + 容差时，才继续往目标前推
  if (dist > chase_distance_ + stop_tolerance_) {
    gx = tx - chase_distance_ * dx / dist;
    gy = ty - chase_distance_ * dy / dist;
  }

  // 追击点区域限制（你新加的需求）
  if (gx < goal_x_min_ || gx > goal_x_max_ || gy < goal_y_min_ || gy > goal_y_max_) {
    RCLCPP_DEBUG(
      logger_,
      "[CalculateChaseGoal] reject by goal area goal=(%.3f, %.3f), area=[%.3f, %.3f] x [%.3f, %.3f]",
      gx, gy, goal_x_min_, goal_x_max_, goal_y_min_, goal_y_max_);
    return BT::NodeStatus::FAILURE;
  }

  goal.pose.position.x = gx;
  goal.pose.position.y = gy;
  goal.pose.position.z = 0.0;

  const double yaw = std::atan2(ty - gy, tx - gx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal.pose.orientation = tf2::toMsg(q);

  setOutput("goal", goal);

  RCLCPP_INFO(
    logger_,
    "[CalculateChaseGoal] cam=(%.3f, %.3f, %.3f) target=(%.3f, %.3f) robot=(%.3f, %.3f) goal=(%.3f, %.3f) dist=%.3f",
    cx, cy, cz, tx, ty, rx, ry, gx, gy, dist);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::CalculateChaseGoal>("CalculateChaseGoal");
}
