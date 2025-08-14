#ifndef CYCLIC_GOAL_NAVIGATOR__GOAL_NAVIGATOR_HPP_
#define CYCLIC_GOAL_NAVIGATOR__GOAL_NAVIGATOR_HPP_

#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <random>
#include <limits>
#include <cmath>
#include <optional>
#include <algorithm>
#include <set>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "cyclic_goal_navigator/costmap_subscriber.hpp"

using namespace std::chrono_literals;

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

    // RViz表示用のマーカーパブリッシャー
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/goal_markers", 10);

    // Nav2の経路を永続化するためのサブスクライバー
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      std::bind(&GoalNavigator::path_callback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_delay_ms),
      std::bind(&GoalNavigator::maybe_send_directional_goal, this));
  }

private:
  void maybe_send_directional_goal()
  {
    if (goal_in_progress_ || is_retrying_) {
      return;  // ゴール実行中または再試行待ち中は何もしない
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

    RCLCPP_INFO(this->get_logger(), "Robot position in %s: (%.3f, %.3f)", source_frame.c_str(),
      robot_x, robot_y);
    RCLCPP_INFO(this->get_logger(), "Robot yaw: %.3f rad (%.1f deg)", robot_yaw,
      robot_yaw * 180.0 / M_PI);

    // No.1: 前方90度を5度刻みで探索（前方方向から左右45度）
    auto forward_cell = find_best_forward_goal_detailed(cells, robot_x, robot_y, robot_yaw);

    if (forward_cell.has_value()) {
      double dist = std::sqrt(forward_cell->first * forward_cell->first +
                             forward_cell->second * forward_cell->second);

      RCLCPP_INFO(this->get_logger(), "No.1: Forward cell found at (%.3f, %.3f), distance: %.3f",
        forward_cell->first, forward_cell->second, dist);

      if (dist >= 1.0) {  // 1m以上の場合、前方ゴールを採用
        // レイキャストで遮蔽チェック（前方はより厳しく50）
        if (!costmap_subscriber_->is_line_free(robot_x, robot_y,
                                               forward_cell->first, forward_cell->second,
                                               /*occ_threshold=*/50,
                                               /*block_on_unknown=*/true))
        {
          RCLCPP_INFO(this->get_logger(), "No.1: Blocked by costmap line-of-sight -> trying No.2");
        } else {
          RCLCPP_INFO(this->get_logger(), "No.1: Forward goal selected (LOS OK)");
          send_nav2_goal(*forward_cell, source_frame);
          return;
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "No.1: Forward goal too close (%.3fm < 1.0m), trying No.2",
          dist);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No.1: No forward cell found, trying No.2");
    }

    // No.2: 左右90度コーン（真横から±45度）の最遠セルを探索（壁チェック付き）
    auto side_cell = find_best_side_goal_with_los_check(cells, robot_x, robot_y, robot_yaw);

    if (side_cell.has_value()) {
      RCLCPP_INFO(this->get_logger(), "No.2: Side goal selected at (%.3f, %.3f)",
        side_cell->first, side_cell->second);
      send_nav2_goal(*side_cell, source_frame);
    } else {
      RCLCPP_WARN(this->get_logger(), "No suitable unblocked goal found in any direction");
    }
  }

  std::optional<std::pair<double, double>> find_best_forward_goal_detailed(
    const std::vector<std::pair<double, double>> & cells,
    double robot_x, double robot_y, double robot_yaw)
  {
    // 前方90度を5度刻みで探索（前方方向±45度）
    std::vector<std::pair<double, double>> all_forward_candidates;

    for (double angle_offset = -45.0; angle_offset <= 45.0; angle_offset += 5.0) {
      double angle = robot_yaw + (angle_offset * M_PI / 180.0);
      auto sector_candidates = find_candidates_in_sector(
        cells, robot_x, robot_y, angle, M_PI / 36, 5);  // 5度の扇形で各5個
      all_forward_candidates.insert(all_forward_candidates.end(),
                                   sector_candidates.begin(),
                                   sector_candidates.end());
    }

    RCLCPP_INFO(this->get_logger(), "No.1: Found %zu forward candidates from ±45 degrees",
                all_forward_candidates.size());

    // 重複を除去
    std::set<std::pair<int, int>> seen_cells;
    std::vector<std::pair<double, std::pair<double, double>>> forward_candidates;

    for (const auto & cell : all_forward_candidates) {
      int grid_x = static_cast<int>(cell.first / 0.1);
      int grid_y = static_cast<int>(cell.second / 0.1);

      if (seen_cells.find({grid_x, grid_y}) == seen_cells.end()) {
        seen_cells.insert({grid_x, grid_y});
        double dx = cell.first - robot_x;
        double dy = cell.second - robot_y;
        double dist2 = dx * dx + dy * dy;
        forward_candidates.push_back({dist2, cell});
      }
    }

    // 距離の降順でソート（最も遠いものから）
    std::sort(forward_candidates.begin(), forward_candidates.end(),
      [](const auto & a, const auto & b) {return a.first > b.first;});

    // 最も遠い候補から順にゴール妥当性と経路計画をチェック
    for (const auto & [dist2, candidate] : forward_candidates) {
      if (!costmap_subscriber_->is_goal_valid(candidate.first, candidate.second)) {
        RCLCPP_INFO(this->get_logger(),
          "No.1: Forward candidate at (%.3f, %.3f) rejected (invalid goal area)",
                    candidate.first, candidate.second);
        continue;
      }

      if (!is_goal_reachable(candidate.first, candidate.second)) {
        RCLCPP_INFO(this->get_logger(),
          "No.1: Forward candidate at (%.3f, %.3f) rejected (unreachable)",
                    candidate.first, candidate.second);
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
        "No.1: Valid and reachable forward candidate selected at (%.3f, %.3f)",
                  candidate.first, candidate.second);
      return candidate;
    }

    return std::nullopt;
  }

  std::optional<std::pair<double, double>> find_farthest_in_sector(
    const std::vector<std::pair<double, double>> & cells,
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

  // 複数の候補を距離順で返す新しい関数
  std::vector<std::pair<double, double>> find_candidates_in_sector(
    const std::vector<std::pair<double, double>> & cells,
    double robot_x, double robot_y, double center_yaw, double half_angle,
    size_t max_candidates = 10)
  {
    std::vector<std::pair<double, std::pair<double, double>>> candidates_with_dist;

    for (const auto & cell : cells) {
      double dx = cell.first - robot_x;
      double dy = cell.second - robot_y;
      double cell_angle = std::atan2(dy, dx);
      double angle_diff = std::abs(normalize_angle(cell_angle - center_yaw));

      if (angle_diff <= half_angle) {
        double dist2 = dx * dx + dy * dy;
        // 近すぎるセルは除外（0.3m以上）
        if (dist2 >= 0.3 * 0.3) {
          candidates_with_dist.push_back({dist2, cell});
        }
      }
    }

    // 距離の降順でソート
    std::sort(candidates_with_dist.begin(), candidates_with_dist.end(),
      [](const auto & a, const auto & b) {return a.first > b.first;});

    // 上位max_candidates個を返す
    std::vector<std::pair<double, double>> result;
    for (size_t i = 0; i < std::min(max_candidates, candidates_with_dist.size()); ++i) {
      result.push_back(candidates_with_dist[i].second);
    }

    return result;
  }

  std::optional<std::pair<double, double>> find_best_side_goal_with_los_check(
    const std::vector<std::pair<double, double>> & cells,
    double robot_x, double robot_y, double robot_yaw)
  {
    // 左右90度±45度の範囲を5度刻みで探索
    std::vector<std::pair<double, double>> all_direction_candidates;

    // 左側: 90度を中心に±45度 (45度から135度)
    for (double angle_offset = -45.0; angle_offset <= 45.0; angle_offset += 5.0) {
      double angle = robot_yaw + M_PI / 2 + (angle_offset * M_PI / 180.0);  // 左90度 + オフセット
      auto sector_candidates = find_candidates_in_sector(
        cells, robot_x, robot_y, angle, M_PI / 36, 3);  // 5度の扇形で各3個
      all_direction_candidates.insert(all_direction_candidates.end(),
                                    sector_candidates.begin(),
                                    sector_candidates.end());
    }

    // 右側: -90度を中心に±45度 (-135度から-45度)
    for (double angle_offset = -45.0; angle_offset <= 45.0; angle_offset += 5.0) {
      double angle = robot_yaw - M_PI / 2 + (angle_offset * M_PI / 180.0);  // 右90度 + オフセット
      auto sector_candidates = find_candidates_in_sector(
        cells, robot_x, robot_y, angle, M_PI / 36, 3);  // 5度の扇形で各3個
      all_direction_candidates.insert(all_direction_candidates.end(),
                                    sector_candidates.begin(),
                                    sector_candidates.end());
    }

    RCLCPP_INFO(this->get_logger(), "No.2: Found %zu candidates from left/right 90±45 degrees",
                all_direction_candidates.size());

    // 重複を除去（同じ位置の候補を削除）
    std::set<std::pair<int, int>> seen_cells;
    std::vector<std::pair<double, std::pair<double, double>>> all_candidates;

    for (const auto & cell : all_direction_candidates) {
      // グリッド座標に変換して重複チェック
      int grid_x = static_cast<int>(cell.first / 0.1);  // 10cmグリッド
      int grid_y = static_cast<int>(cell.second / 0.1);

      if (seen_cells.find({grid_x, grid_y}) == seen_cells.end()) {
        seen_cells.insert({grid_x, grid_y});
        double dx = cell.first - robot_x;
        double dy = cell.second - robot_y;
        double dist2 = dx * dx + dy * dy;
        all_candidates.push_back({dist2, cell});
      }
    }

    // 距離の降順でソート
    std::sort(all_candidates.begin(), all_candidates.end(),
      [](const auto & a, const auto & b) {return a.first > b.first;});

    RCLCPP_INFO(this->get_logger(), "No.2: Testing %zu total side candidates",
      all_candidates.size());

    // 各候補に対して壁チェックを行い、最初にパスしたものを返す
    int tested_count = 0;
    for (const auto & [dist2, cell] : all_candidates) {
      tested_count++;
      // コスト閾値を90に引き上げ（さらに寛容）、経路の80%以上がクリアなら許可
      bool line_free = costmap_subscriber_->is_line_free(robot_x, robot_y,
                                           cell.first, cell.second,
                                           /*occ_threshold=*/90,
                                           /*block_on_unknown=*/false);  // unknownセルは通行可能とみなす

      if (line_free && costmap_subscriber_->is_goal_valid(cell.first, cell.second) &&
        is_goal_reachable(cell.first, cell.second))
      {
        RCLCPP_INFO(this->get_logger(),
          "Found valid and reachable side goal at (%.3f, %.3f) distance: %.3f (LOS OK, tested %d candidates)",
                    cell.first, cell.second, std::sqrt(dist2), tested_count);
        return cell;
      } else {
        // 部分的に通行可能かチェック（80%以上クリアなら許可）
        // より単純なチェック：中間点がクリアか確認
        double dx = cell.first - robot_x;
        double dy = cell.second - robot_y;
        double distance = std::sqrt(dist2);

        // より寛容な部分チェック：経路の50%がクリアなら許可
        int clear_segments = 0;
        int total_segments = 4;

        for (double ratio = 0.25; ratio <= 1.0; ratio += 0.25) {
          double check_x = robot_x + dx * ratio;
          double check_y = robot_y + dy * ratio;

          // より短い距離での部分チェック
          if (costmap_subscriber_->is_line_free(robot_x, robot_y, check_x, check_y, 95, false)) {
            clear_segments++;
          }
        }

        double clear_ratio = static_cast<double>(clear_segments) / total_segments;
        if (clear_ratio >= 0.5 && costmap_subscriber_->is_goal_valid(cell.first, cell.second) &&
          is_goal_reachable(cell.first, cell.second))      // 50%以上クリア + ゴール妥当性 + 到達可能性
        {
          RCLCPP_INFO(this->get_logger(),
            "Found partially clear and reachable side goal at (%.3f, %.3f) distance: %.3f (%.0f%% clear, tested %d candidates)",
                      cell.first, cell.second, distance, clear_ratio * 100, tested_count);
          return cell;
        } else {
          RCLCPP_INFO(this->get_logger(),
            "Side candidate %d at (%.3f, %.3f) distance: %.3f blocked (%.0f%% clear)",
                      tested_count, cell.first, cell.second, distance, clear_ratio * 100);
        }
      }
    }

    RCLCPP_WARN(this->get_logger(), "No.2: All %d side candidates were blocked", tested_count);

    // 緊急脱出モード：全候補がブロックされた場合は最も近い候補を強制選択
    if (!all_candidates.empty()) {
      auto emergency_goal = all_candidates[all_candidates.size() - 1].second; // 最も近い候補
      double emergency_distance = std::sqrt(all_candidates[all_candidates.size() - 1].first);
      RCLCPP_WARN(this->get_logger(),
        "EMERGENCY MODE: Selecting closest goal at (%.3f, %.3f) distance: %.3f",
                  emergency_goal.first, emergency_goal.second, emergency_distance);
      return emergency_goal;
    }

    return std::nullopt;
  }

  double normalize_angle(double angle)
  {
    // 角度を-π～πに正規化
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }

  void send_nav2_goal(const std::pair<double, double> & cell, const std::string & source_frame)
  {
    // source_frame上のセル座標をPointStampedで表現
    geometry_msgs::msg::PointStamped pt_src, pt_dst;
    pt_src.header.frame_id = source_frame;
    pt_src.header.stamp.sec = 0;
    pt_src.header.stamp.nanosec = 0;  // Time=0 -> latest transform
    pt_src.point.x = cell.first;
    pt_src.point.y = cell.second;
    pt_src.point.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Point in %s: (%.3f, %.3f)", source_frame.c_str(),
      pt_src.point.x, pt_src.point.y);

    // source_frame -> target_frame へ座標変換
    try {
      if (!tf_buffer_->canTransform(target_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.5)))
      {
        RCLCPP_WARN(this->get_logger(), "TF not available between %s and %s", source_frame.c_str(),
          target_frame_.c_str());
        return;
      }
      tf_buffer_->transform(pt_src, pt_dst, target_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Point in %s: (%.3f, %.3f)", target_frame_.c_str(),
      pt_dst.point.x, pt_dst.point.y);

    // ロボットのmap座標も確認
    geometry_msgs::msg::TransformStamped tf_bl_map;
    double robot_x = 0.0, robot_y = 0.0;
    try {
      tf_bl_map = tf_buffer_->lookupTransform(
        target_frame_, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
      robot_x = tf_bl_map.transform.translation.x;
      robot_y = tf_bl_map.transform.translation.y;
      RCLCPP_INFO(this->get_logger(), "Robot in %s: (%.3f, %.3f)", target_frame_.c_str(),
        robot_x, robot_y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Robot TF lookup failed: %s", ex.what());
    }

    // 現在位置からゴール位置への方向を計算
    double dx = pt_dst.point.x - robot_x;
    double dy = pt_dst.point.y - robot_y;
    double goal_yaw = std::atan2(dy, dx);

    // ヨー角からクォータニオンに変換
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, goal_yaw);

    // NavigateToPose ゴール作成
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.header.frame_id = target_frame_;
    goal_msg.pose.pose.position.x = pt_dst.point.x;
    goal_msg.pose.pose.position.y = pt_dst.point.y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = goal_quat.x();
    goal_msg.pose.pose.orientation.y = goal_quat.y();
    goal_msg.pose.pose.orientation.z = goal_quat.z();
    goal_msg.pose.pose.orientation.w = goal_quat.w();

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
            {
              RCLCPP_WARN(this->get_logger(), "Goal aborted. Will retry in 3 seconds.");
              is_retrying_ = true;

            // 既存のretry_timerがあればキャンセル
              if (retry_timer_) {
                retry_timer_->cancel();
              }

            // 3秒後に再試行（ワンショット）
              retry_timer_ = this->create_wall_timer(
              std::chrono::seconds(3),
                [this]() {
                  is_retrying_ = false;
                  retry_timer_->cancel(); // タイマーを停止
                  retry_timer_ = nullptr;
                  this->maybe_send_directional_goal();
              });
              break;
            }
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code.");
            break;
        }
      };

    RCLCPP_INFO(this->get_logger(),
      "Sending NavigateToPose to directional goal (%.3f, %.3f) with yaw %.2f rad in %s",
      goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_yaw,
      target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "=== End Directional Selection ===");

    // RVizにゴールマーカーを表示（姿勢も含む）
    publish_goal_marker(goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_yaw,
      target_frame_);

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
  bool is_retrying_ {false};  // 再試行中フラグ

  // RViz表示用
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  int marker_id_ {0};

  // 経路永続化用
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  int path_marker_id_ {1000};  // ゴールマーカーと区別するため1000から開始

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

private:
  bool is_goal_reachable(double goal_x, double goal_y)
  {
    // CostmapSubscriberの詳細チェック機能を使用
    bool is_safe = costmap_subscriber_->is_goal_area_safe(goal_x, goal_y);

    if (!is_safe) {
      RCLCPP_INFO(this->get_logger(),
        "Goal at (%.3f, %.3f) rejected - unsafe area based on costmap",
                  goal_x, goal_y);
    }

    return is_safe;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;  // 空の経路は無視
    }

    // PathをLineStripマーカーに変換
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto path_marker = visualization_msgs::msg::Marker();

    path_marker.header.frame_id = msg->header.frame_id;
    path_marker.header.stamp = this->get_clock()->now();
    path_marker.ns = "persistent_paths";
    path_marker.id = path_marker_id_++;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;

    // 線の設定
    path_marker.scale.x = 0.05;  // 線の太さ 5cm
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;   // 緑色
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.8;   // 透明度

    // Pathの全ポイントをLineStripに追加
    for (const auto & pose_stamped : msg->poses) {
      geometry_msgs::msg::Point point;
      point.x = pose_stamped.pose.position.x;
      point.y = pose_stamped.pose.position.y;
      point.z = 0.1;  // 少し浮かせて表示
      path_marker.points.push_back(point);
    }

    marker_array.markers.push_back(path_marker);
    marker_pub_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Persistent path published with %zu points (ID: %d)",
                msg->poses.size(), path_marker.id);
  }

  void publish_goal_marker(double x, double y, double yaw, const std::string & frame_id)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // ゴール位置のマーカー（赤い矢印）
    auto goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = frame_id;
    goal_marker.header.stamp = this->get_clock()->now();
    goal_marker.ns = "goal_position";
    goal_marker.id = marker_id_++;
    goal_marker.type = visualization_msgs::msg::Marker::ARROW;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;

    goal_marker.pose.position.x = x;
    goal_marker.pose.position.y = y;
    goal_marker.pose.position.z = 0.1;  // 少し浮かせる

    // ヨー角をクォータニオンに変換してマーカーの向きを設定
    tf2::Quaternion marker_quat;
    marker_quat.setRPY(0.0, 0.0, yaw);
    goal_marker.pose.orientation.x = marker_quat.x();
    goal_marker.pose.orientation.y = marker_quat.y();
    goal_marker.pose.orientation.z = marker_quat.z();
    goal_marker.pose.orientation.w = marker_quat.w();

    goal_marker.scale.x = 0.5;  // 矢印の長さ
    goal_marker.scale.y = 0.1;  // 矢印の幅
    goal_marker.scale.z = 0.1;  // 矢印の高さ

    goal_marker.color.r = 1.0;  // 赤色
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 0.8;  // 透明度

    marker_array.markers.push_back(goal_marker);

    // ゴール位置のテキストラベル
    auto text_marker = visualization_msgs::msg::Marker();
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = "goal_text";
    text_marker.id = marker_id_++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = x;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = 0.3;  // 矢印より上に表示
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.2;  // テキストサイズ

    text_marker.color.r = 1.0;  // 白色
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    // 座標を表示
    char text_buffer[100];
    snprintf(text_buffer, sizeof(text_buffer), "Goal\n(%.2f, %.2f)", x, y);
    text_marker.text = text_buffer;

    marker_array.markers.push_back(text_marker);

    // マーカーをパブリッシュ
    marker_pub_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Published goal marker at (%.3f, %.3f) in %s frame", x, y,
      frame_id.c_str());
  }
};

#endif  // CYCLIC_GOAL_NAVIGATOR__GOAL_NAVIGATOR_HPP_
