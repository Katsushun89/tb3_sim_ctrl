#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

class CostmapSubscriber : public rclcpp::Node
{
public:
  CostmapSubscriber()
  : Node("costmap_subscriber")
  {
    // rosparamからしきい値を取得（存在しなければデフォルト5）
    this->declare_parameter<int>("cost_threshold", 5);
    cost_threshold_ = this->get_parameter("cost_threshold").as_int();

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap",
      10,
      std::bind(&CostmapSubscriber::costmap_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&CostmapSubscriber::timer_callback, this));
  }

private:
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    cost_cells_.clear();

    // costmap情報の取得
    auto width = msg->info.width;
    //auto height = msg->info.height; //unused
    auto resolution = msg->info.resolution;
    auto origin_x = msg->info.origin.position.x;
    auto origin_y = msg->info.origin.position.y;
    const auto & data = msg->data;

    for (size_t i = 0; i < data.size(); ++i) {
      int cost = data[i];
      // costが-1(未知)以外でしきい値以下のセルを抽出
      if (cost >= 0 && cost <= cost_threshold_) {
        int x_idx = i % width;
        int y_idx = i / width;
        double x = origin_x + (x_idx + 0.5) * resolution;  // セル中心座標
        double y = origin_y + (y_idx + 0.5) * resolution;
        cost_cells_.emplace_back(std::make_pair(x, y));
      }
    }
  }

  void timer_callback()
  {
    // 1秒ごとに該当コストセルの座標をデバッグ表示
    RCLCPP_INFO(this->get_logger(), "Cells with cost <= %d count: %zu", cost_threshold_, cost_cells_.size());

    for (const auto & cell : cost_cells_) {
      RCLCPP_INFO(this->get_logger(), "  Cell at (%.3f, %.3f)", cell.first, cell.second);
    }
  }

  int cost_threshold_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // しきい値以下のコストセル座標の配列（x, y）
  std::vector<std::pair<double, double>> cost_cells_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CostmapSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
