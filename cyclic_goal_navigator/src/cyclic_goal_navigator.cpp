#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
    RCLCPP_INFO(this->get_logger(), "Cells with cost <= %d count: %zu (frame: %s)",
      cost_threshold_, low_cost_cells_xy_.size(), costmap_frame_id_.c_str());
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
      std::bind(&GoalNavigator::maybe_send_random_goal, this));
  }

private:
  void maybe_send_random_goal()
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

    // ランダムに1セル選択
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, cells.size() - 1);
    const auto & cell = cells[dist(gen)];

    // source_frame -> target_frame へ座標変換
    geometry_msgs::msg::PointStamped point_src, point_dst;
    point_src.header.stamp.sec = 0;
    point_src.header.stamp.nanosec = 0;  // Time=0 -> latest transform
    point_src.header.frame_id = source_frame;
    point_src.point.x = cell.first;
    point_src.point.y = cell.second;
    point_src.point.z = 0.0;

    try {
      if (!tf_buffer_->canTransform(target_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
        RCLCPP_WARN(this->get_logger(), "TF not available between %s and %s", source_frame.c_str(), target_frame_.c_str());
        return;
      }
      tf_buffer_->transform(point_src, point_dst, target_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    // NavigateToPose ゴール作成
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.header.frame_id = target_frame_;
    goal_msg.pose.pose.position.x = point_dst.point.x;
    goal_msg.pose.pose.position.y = point_dst.point.y;
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
            this->maybe_send_random_goal();
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

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose to (%.3f, %.3f) in %s",
      goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, target_frame_.c_str());

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
