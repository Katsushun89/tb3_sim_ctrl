#ifndef CYCLIC_GOAL_NAVIGATOR__COSTMAP_SUBSCRIBER_HPP_
#define CYCLIC_GOAL_NAVIGATOR__COSTMAP_SUBSCRIBER_HPP_

#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

class CostmapSubscriber : public rclcpp::Node
{
public:
  CostmapSubscriber()
  : Node("costmap_subscriber")
  {
    // パラメータ
    this->declare_parameter<int>("cost_threshold", 1);
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

  bool is_goal_valid(double goal_x, double goal_y)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_grid_) {return false;}

    auto & g = *last_grid_;
    const double res = g.info.resolution;
    const double ox = g.info.origin.position.x;
    const double oy = g.info.origin.position.y;
    const int W = g.info.width;
    const int H = g.info.height;

    // ゴール位置をマップ座標に変換
    int mx = static_cast<int>(std::floor((goal_x - ox) / res));
    int my = static_cast<int>(std::floor((goal_y - oy) / res));

    // 境界チェック
    if (mx < 0 || mx >= W || my < 0 || my >= H) {
      return false;  // マップ外はNG
    }

    // ゴール周辺（3x3）のセルをチェック
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        int check_x = mx + dx;
        int check_y = my + dy;

        // 境界チェック
        if (check_x < 0 || check_x >= W || check_y < 0 || check_y >= H) {
          return false;  // 周辺がマップ外はNG
        }

        // 周辺セルのコストチェック
        int idx = check_y * W + check_x;
        int8_t cost = g.data[idx];

        // 周辺に高コスト（障害物）があるかチェック
        if (cost >= 80) {  // 80以上は危険
          return false;
        }

        // unknownセル（-1）が多すぎる場合も危険
        if (cost < 0) {
          return false;
        }
      }
    }

    return true;  // ゴールとして妥当
  }

  bool is_goal_area_safe(double goal_x, double goal_y)
  {
    // ゴール周辺の安全性を詳細にチェック
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_grid_) {return true;}

    auto & g = *last_grid_;
    const double res = g.info.resolution;
    const double ox = g.info.origin.position.x;
    const double oy = g.info.origin.position.y;
    const int W = g.info.width;
    const int H = g.info.height;

    // ゴール位置をマップ座標に変換
    int mx = static_cast<int>(std::floor((goal_x - ox) / res));
    int my = static_cast<int>(std::floor((goal_y - oy) / res));

    // 境界チェック
    if (mx < 2 || mx >= W - 2 || my < 2 || my >= H - 2) {
      return false;  // マップ境界に近すぎる
    }

    // ゴール周辺（5x5）のセルをチェック
    int high_cost_count = 0;
    int unknown_count = 0;
    int total_cells = 0;

    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        int check_x = mx + dx;
        int check_y = my + dy;

        if (check_x < 0 || check_x >= W || check_y < 0 || check_y >= H) {
          return false;  // 範囲外
        }

        int idx = check_y * W + check_x;
        int8_t cost = g.data[idx];
        total_cells++;

        if (cost < 0) {  // unknown
          unknown_count++;
        } else if (cost >= 70) {  // 高コスト
          high_cost_count++;
        }
      }
    }

    // 判定条件
    double high_cost_ratio = static_cast<double>(high_cost_count) / total_cells;
    double unknown_ratio = static_cast<double>(unknown_count) / total_cells;

    // 高コストセルが40%以上、またはunknownセルが60%以上なら危険
    return  high_cost_ratio <= 0.4 && unknown_ratio <= 0.6;
  }

  bool is_line_free(
    double x0, double y0, double x1, double y1,
    int occ_threshold = 50, bool block_on_unknown = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_grid_) {return false;}

    auto & g = *last_grid_;
    const int W = g.info.width;
    const int H = g.info.height;
    const double res = g.info.resolution;
    const double ox = g.info.origin.position.x;
    const double oy = g.info.origin.position.y;

    auto world_to_map = [&](double wx, double wy, int & mx, int & my)->bool {
        mx = static_cast<int>(std::floor((wx - ox) / res));
        my = static_cast<int>(std::floor((wy - oy) / res));
        return  mx >= 0 && mx < static_cast<int>(W) && my >= 0 && my < static_cast<int>(H);
      };

    int x0i, y0i, x1i, y1i;
    if (!world_to_map(x0, y0, x0i, y0i) || !world_to_map(x1, y1, x1i, y1i)) {
      return false; // 枠外は通せない扱い
    }

    auto idx = [&](int x, int y){return y * static_cast<int>(W) + x;};

    // Bresenham
    int dx = std::abs(x1i - x0i), sx = x0i < x1i ? 1 : -1;
    int dy = -std::abs(y1i - y0i), sy = y0i < y1i ? 1 : -1;
    int err = dx + dy; // err = dx + dy (※dyは負)
    int x = x0i, y = y0i;

    while (true) {
      const int8_t cost = g.data[idx(x, y)];
      if ((block_on_unknown && cost < 0) || cost >= occ_threshold) {
        return false; // 遮蔽あり
      }
      if (x == x1i && y == y1i) {break;}
      const int e2 = 2 * err;
      if (e2 >= dy) {err += dy; x += sx;}
      if (e2 <= dx) {err += dx; y += sy;}
    }
    return true; // 直線上に障害なし
  }

  // テスト用のgetter
  void set_last_grid_for_test(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_grid_ = grid;
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
    last_grid_ = msg;
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
  nav_msgs::msg::OccupancyGrid::SharedPtr last_grid_;
};

#endif  // CYCLIC_GOAL_NAVIGATOR__COSTMAP_SUBSCRIBER_HPP_
