#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <random>
#include <limits>
#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;

class CostmapSubscriber : public rclcpp::Node
{
public:
  CostmapSubscriber()
  : Node("costmap_subscriber")
  {
    // パラメータ
    this->declare_parameter<int>("cost_threshold", 5);
    this->declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap");
    cost_threshold_ = this->get_parameter("cost_threshold").as_int();
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CostmapSubscriber::costmap_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1s, std::bind(&CostmapSubscriber::timer_callback, this));
  }

  // 最新の低コストセル座標のコピーを取得
  std::vector<std::pair<double, double>> get_low_cost_cells_copy()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return low_cost_cells_xy_;
  }

  // サブスクライブしているコストマップのフレームIDを取得（例: odom/map）
  std::string get_costmap_frame_id()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return costmap_frame_id_;
  }

private:
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    low_cost_cells_xy_.clear();

    const uint32_t width = msg->info.width;
    const double resolution = msg->info.resolution;
    const double origin_x = msg->info.origin.position.x;
    const double origin_y = msg->info.origin.position.y;
    const auto & data = msg->data;
    costmap_frame_id_ = msg->header.frame_id;

    for (size_t i = 0; i < data.size(); ++i) {
      const int cost = data[i];
      // costが-1(未知)以外でしきい値以下のセルを抽出
      if (cost >= 0 && cost <= cost_threshold_) {
        const uint32_t x_idx = static_cast<uint32_t>(i % width);
        const uint32_t y_idx = static_cast<uint32_t>(i / width);
        const double x = origin_x + (static_cast<double>(x_idx) + 0.5) * resolution;  // セル中心座標
        const double y = origin_y + (static_cast<double>(y_idx) + 0.5) * resolution;
        low_cost_cells_xy_.emplace_back(x, y);
      }
    }
  }

  void timer_callback()
  {
    // 1秒ごとに該当コストセルの件数をデバッグ表示
    std::lock_guard<std::mutex> lock(mutex_);
    //RCLCPP_INFO(this->get_logger(), "Cells with cost <= %d count: %zu (frame: %s)",
    //  cost_threshold_, low_cost_cells_xy_.size(), costmap_frame_id_.c_str());
  }

  int cost_threshold_;
  std::string costmap_topic_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 共有データ
  std::mutex mutex_;
  std::vector<std::pair<double, double>> low_cost_cells_xy_;
  std::string costmap_frame_id_;
};

class GoalNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit GoalNavigator(const std::shared_ptr<CostmapSubscriber> & costmap_subscriber)
  : Node("goal_navigator"), costmap_subscriber_(costmap_subscriber)
  {
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    this->declare_parameter<int>("publish_delay_ms", 1000);

    target_frame_ = this->get_parameter("target_frame").as_string();
    action_name_ = this->get_parameter("nav2_action_name").as_string();
    const int publish_delay_ms = this->get_parameter("publish_delay_ms").as_int();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_delay_ms),
      std::bind(&GoalNavigator::maybe_send_directional_goal, this));
  }

private:
  void maybe_send_directional_goal()
  {
    if (goal_in_progress_) {
      return;
    }

    if (!action_client_->wait_for_action_server(100ms)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for Nav2 action server '%s'...", action_name_.c_str());
      return;
    }

    const auto cells = costmap_subscriber_->get_low_cost_cells_copy();
    const std::string source_frame = costmap_subscriber_->get_costmap_frame_id();

    if (cells.empty() || source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for low-cost cells or valid frame... cells=%zu frame='%s'",
        cells.size(), source_frame.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "=== Directional Goal Selection ===");
    RCLCPP_INFO(this->get_logger(), "Source frame: %s", source_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Available cells: %zu", cells.size());

    // ロボットの現在位置と向きを取得
    geometry_msgs::msg::TransformStamped tf_bl_src;
    try {
      tf_bl_src = tf_buffer_->lookupTransform(
        source_frame, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed (robot pose): %s", ex.what());
      return;
    }

    const double robot_x = tf_bl_src.transform.translation.x;
    const double robot_y = tf_bl_src.transform.translation.y;
    
    // ロボットの向き（yaw角）を取得
    double robot_yaw = 0.0;
    try {
      tf2::Quaternion q;
      tf2::fromMsg(tf_bl_src.transform.rotation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      robot_yaw = yaw;
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to extract robot yaw: %s", ex.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Robot position in %s: (%.3f, %.3f)", source_frame.c_str(), robot_x, robot_y);
    RCLCPP_INFO(this->get_logger(), "Robot yaw: %.3f rad (%.1f deg)", robot_yaw, robot_yaw * 180.0 / M_PI);

    // No.1: 前方90度コーン（前方方向から左右45度）の最遠セルを探索
    auto forward_cell = find_farthest_in_sector(cells, robot_x, robot_y, robot_yaw, M_PI/4);
    
    if (forward_cell.has_value()) {
      double dist = std::sqrt(forward_cell->first * forward_cell->first + 
                             forward_cell->second * forward_cell->second);
      
      RCLCPP_INFO(this->get_logger(), "No.1: Forward cell found at (%.3f, %.3f), distance: %.3f", 
        forward_cell->first, forward_cell->second, dist);
      
      if (dist >= 1.0) {  // 1m以上の場合、前方ゴールを採用
        RCLCPP_INFO(this->get_logger(), "No.1: Forward goal selected (distance >= 1.0m)");
        send_nav2_goal(*forward_cell, source_frame);
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "No.1: Forward goal too close (%.3fm < 1.0m), trying No.2", dist);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No.1: No forward cell found, trying No.2");
    }

    // No.2: 左右90度コーン（真横から±45度）の最遠セルを探索
    auto side_cell = find_farthest_in_side_sectors(cells, robot_x, robot_y, robot_yaw);
    
    if (side_cell.has_value()) {
      RCLCPP_INFO(this->get_logger(), "No.2: Side goal selected at (%.3f, %.3f)", 
        side_cell->first, side_cell->second);
      send_nav2_goal(*side_cell, source_frame);
    } else {
      RCLCPP_WARN(this->get_logger(), "No suitable goal found in any direction");
    }
  }

  std::optional<std::pair<double, double>> find_farthest_in_sector(
    const std::vector<std::pair<double, double>>& cells,
    double robot_x, double robot_y, double center_yaw, double half_angle)
  {
    double max_dist2 = 0.0;
    std::optional<std::pair<double, double>> best_cell;

    for (const auto & cell : cells) {
      // ロボット中心からの相対座標
      double dx = cell.first - robot_x;
      double dy = cell.second - robot_y;
      
      // セルの角度を計算（ロボット中心から見た角度）
      double cell_angle = std::atan2(dy, dx);
      double angle_diff = std::abs(normalize_angle(cell_angle - center_yaw));
      
      // 指定範囲内かチェック
      if (angle_diff <= half_angle) {
        double dist2 = dx * dx + dy * dy;
        if (dist2 > max_dist2) {
          max_dist2 = dist2;
          best_cell = cell;
        }
      }
    }
    
    return best_cell;
  }

  std::optional<std::pair<double, double>> find_farthest_in_side_sectors(
    const std::vector<std::pair<double, double>>& cells,
    double robot_x, double robot_y, double robot_yaw)
  {
    // 左側（robot_yaw + π/2）と右側（robot_yaw - π/2）の両方を探索
    auto left_cell = find_farthest_in_sector(cells, robot_x, robot_y, robot_yaw + M_PI/2, M_PI/4);   // 左90度±45度
    auto right_cell = find_farthest_in_sector(cells, robot_x, robot_y, robot_yaw - M_PI/2, M_PI/4);  // 右90度±45度
    
    // 左右で最も遠いセルを選択
    if (left_cell.has_value() && right_cell.has_value()) {
      double left_dx = left_cell->first - robot_x;
      double left_dy = left_cell->second - robot_y;
      double right_dx = right_cell->first - robot_x;
      double right_dy = right_cell->second - robot_y;
      
      double left_dist2 = left_dx * left_dx + left_dy * left_dy;
      double right_dist2 = right_dx * right_dx + right_dy * right_dy;
      
      return (left_dist2 > right_dist2) ? left_cell : right_cell;
    } else if (left_cell.has_value()) {
      return left_cell;
    } else if (right_cell.has_value()) {
      return right_cell;
    }
    
    return std::nullopt;
  }

  double normalize_angle(double angle)
  {
    // 角度を-π～πに正規化
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void send_nav2_goal(const std::pair<double, double>& cell, const std::string& source_frame)
  {
    // source_frame上のセル座標をPointStampedで表現
    geometry_msgs::msg::PointStamped pt_src, pt_dst;
    pt_src.header.frame_id = source_frame;
    pt_src.header.stamp.sec = 0;
    pt_src.header.stamp.nanosec = 0;  // Time=0 -> latest transform
    pt_src.point.x = cell.first;
    pt_src.point.y = cell.second;
    pt_src.point.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Point in %s: (%.3f, %.3f)", source_frame.c_str(), pt_src.point.x, pt_src.point.y);

    // source_frame -> target_frame へ座標変換
    try {
      if (!tf_buffer_->canTransform(target_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
        RCLCPP_WARN(this->get_logger(), "TF not available between %s and %s", source_frame.c_str(), target_frame_.c_str());
        return;
      }
      tf_buffer_->transform(pt_src, pt_dst, target_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Point in %s: (%.3f, %.3f)", target_frame_.c_str(), pt_dst.point.x, pt_dst.point.y);

    // ロボットのmap座標も確認
    geometry_msgs::msg::TransformStamped tf_bl_map;
    try {
      tf_bl_map = tf_buffer_->lookupTransform(
        target_frame_, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
      RCLCPP_INFO(this->get_logger(), "Robot in %s: (%.3f, %.3f)", target_frame_.c_str(), 
        tf_bl_map.transform.translation.x, tf_bl_map.transform.translation.y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Robot TF lookup failed: %s", ex.what());
    }

    // NavigateToPose ゴール作成
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.header.frame_id = target_frame_;
    goal_msg.pose.pose.position.x = pt_dst.point.x;
    goal_msg.pose.pose.position.y = pt_dst.point.y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleNav2> handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal rejected");
          goal_in_progress_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted");
          goal_in_progress_ = true;
        }
      };

    send_goal_options.feedback_callback =
      [this](
        std::shared_ptr<GoalHandleNav2>,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        (void)feedback; // 拡張用
      };

    send_goal_options.result_callback =
      [this](const GoalHandleNav2::WrappedResult & result) {
        goal_in_progress_ = false;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached. Scheduling next goal.");
            // 次のゴールをすぐに再送
            this->maybe_send_directional_goal();
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal aborted. Will retry later.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code.");
            break;
        }
      };

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose to directional goal (%.3f, %.3f) in %s",
      goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "=== End Directional Selection ===");

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Inputs
  std::shared_ptr<CostmapSubscriber> costmap_subscriber_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::string action_name_;

  std::string target_frame_;
  bool goal_in_progress_ {false};

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto costmap_node = std::make_shared<CostmapSubscriber>();
  auto navigator_node = std::make_shared<GoalNavigator>(costmap_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(costmap_node);
  exec.add_node(navigator_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
