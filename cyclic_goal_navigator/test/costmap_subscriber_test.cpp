#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "cyclic_goal_navigator/costmap_subscriber.hpp"

class CostmapSubscriberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<CostmapSubscriber>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  // テスト用のOccupancyGridメッセージを作成
  nav_msgs::msg::OccupancyGrid::SharedPtr create_test_grid(
    int width, int height, double resolution,
    double origin_x = 0.0, double origin_y = 0.0)
  {
    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = "test_frame";
    grid->info.width = width;
    grid->info.height = height;
    grid->info.resolution = resolution;
    grid->info.origin.position.x = origin_x;
    grid->info.origin.position.y = origin_y;
    grid->info.origin.position.z = 0.0;

    // データを初期化（全て0 = 低コスト）
    grid->data.resize(width * height, 0);

    return grid;
  }

  // グリッドに障害物を設定
  void set_obstacle(
    nav_msgs::msg::OccupancyGrid::SharedPtr grid,
    int x, int y, int8_t cost = 100)
  {
    if (x >= 0 && x < static_cast<int>(grid->info.width) &&
      y >= 0 && y < static_cast<int>(grid->info.height))
    {
      grid->data[y * grid->info.width + x] = cost;
    }
  }

  // グリッドにunknownセルを設定
  void set_unknown(
    nav_msgs::msg::OccupancyGrid::SharedPtr grid,
    int x, int y)
  {
    if (x >= 0 && x < static_cast<int>(grid->info.width) &&
      y >= 0 && y < static_cast<int>(grid->info.height))
    {
      grid->data[y * grid->info.width + x] = -1;
    }
  }

  std::shared_ptr<CostmapSubscriber> node_;
};

// テスト: is_goal_valid - データがない場合
TEST_F(CostmapSubscriberTest, IsGoalValid_NoData)
{
  // データがない場合はfalseを返すべき
  EXPECT_FALSE(node_->is_goal_valid(0.5, 0.5));
  EXPECT_FALSE(node_->is_goal_valid(100.0, 100.0));
  EXPECT_FALSE(node_->is_goal_valid(-100.0, -100.0));
}

// テスト: is_goal_valid - グリッドデータありの場合
TEST_F(CostmapSubscriberTest, IsGoalValid_WithData)
{
  // 5x5のグリッドを作成（resolution=0.1m）
  auto grid = create_test_grid(5, 5, 0.1, 0.0, 0.0);

  // 中央(2,2)は安全、周辺に障害物を配置
  set_obstacle(grid, 0, 0, 90);  // 高コスト
  set_obstacle(grid, 4, 4, 90);  // 高コスト
  set_unknown(grid, 0, 1);       // unknown

  node_->set_last_grid_for_test(grid);

  // 中央のセル(2,2) -> world座標(0.25, 0.25)は安全
  EXPECT_TRUE(node_->is_goal_valid(0.25, 0.25));

  // 障害物のあるセル(0,0) -> world座標(0.05, 0.05)は危険
  EXPECT_FALSE(node_->is_goal_valid(0.05, 0.05));

  // unknownセルのあるセル(0,1) -> world座標(0.05, 0.15)は危険
  EXPECT_FALSE(node_->is_goal_valid(0.05, 0.15));

  // マップ外は無効
  EXPECT_FALSE(node_->is_goal_valid(1.0, 1.0));
  EXPECT_FALSE(node_->is_goal_valid(-1.0, -1.0));
}

// テスト: is_goal_area_safe - データがない場合
TEST_F(CostmapSubscriberTest, IsGoalAreaSafe_NoData)
{
  // データがない場合はtrueを返す（デフォルト安全）
  EXPECT_TRUE(node_->is_goal_area_safe(0.5, 0.5));
}

// テスト: is_goal_area_safe - 5x5エリア安全性チェックロジック
TEST_F(CostmapSubscriberTest, IsGoalAreaSafe_WithData)
{
  // 7x7のグリッドを作成（中央5x5をチェックするため）
  auto grid = create_test_grid(7, 7, 0.1, 0.0, 0.0);

  // 中央(3,3)周辺の5x5エリアに様々なコストを配置
  set_obstacle(grid, 2, 2, 80);  // 高コスト（>= 70）
  set_obstacle(grid, 2, 3, 80);  // 高コスト
  set_unknown(grid, 4, 4);       // unknown
  set_unknown(grid, 4, 3);       // unknown

  node_->set_last_grid_for_test(grid);

  // 中央(3,3) -> world座標(0.35, 0.35)
  // 高コスト: 2/25 = 8% (40%以下なのでOK)
  // unknown: 2/25 = 8% (60%以下なのでOK)
  EXPECT_TRUE(node_->is_goal_area_safe(0.35, 0.35));

  // より多くの高コストセルを追加して40%を超える
  for (int i = 1; i <= 5; ++i) {
    for (int j = 1; j <= 5; ++j) {
      if (i <= 3 && j <= 3) {  // 左上の9セルを高コストに
        set_obstacle(grid, i, j, 80);
      }
    }
  }

  node_->set_last_grid_for_test(grid);

  // 高コスト: 9/25 = 36% (まだ40%以下なのでOK)
  EXPECT_TRUE(node_->is_goal_area_safe(0.35, 0.35));

  // さらに高コストセルを追加して40%を超える
  set_obstacle(grid, 4, 1, 80);
  set_obstacle(grid, 4, 2, 80);
  set_obstacle(grid, 5, 1, 80);

  node_->set_last_grid_for_test(grid);

  // 高コスト: 12/25 = 48% (40%を超えるので危険)
  EXPECT_FALSE(node_->is_goal_area_safe(0.35, 0.35));
}

// テスト: is_line_free - データがない場合
TEST_F(CostmapSubscriberTest, IsLineFree_NoData)
{
  // データがない場合はfalseを返す
  EXPECT_FALSE(node_->is_line_free(0.0, 0.0, 1.0, 1.0));
}

// テスト: is_line_free - Bresenhamアルゴリズムによる直線経路チェック
TEST_F(CostmapSubscriberTest, IsLineFree_WithData)
{
  // 10x10のグリッドを作成
  auto grid = create_test_grid(10, 10, 0.1, 0.0, 0.0);

  // 対角線上に障害物を配置 (1,1), (2,2), (3,3)
  set_obstacle(grid, 1, 1, 60);
  set_obstacle(grid, 2, 2, 60);
  set_obstacle(grid, 3, 3, 60);

  // unknownセルも配置
  set_unknown(grid, 5, 5);

  node_->set_last_grid_for_test(grid);

  // (0,0) から (4,4) への対角線は障害物を通る
  EXPECT_FALSE(node_->is_line_free(0.05, 0.05, 0.45, 0.45, 50, true));

  // 閾値を上げれば通れる
  EXPECT_TRUE(node_->is_line_free(0.05, 0.05, 0.45, 0.45, 70, true));

  // unknownセルを通る経路: (0,0) から (6,6)
  EXPECT_FALSE(node_->is_line_free(0.05, 0.05, 0.65, 0.65, 50, true));

  // unknownをブロックしない設定（閾値を上げて障害物を回避）
  EXPECT_TRUE(node_->is_line_free(0.05, 0.05, 0.65, 0.65, 70, false));

  // 障害物のない経路: (0,0) から (0,9)（縦方向）
  EXPECT_TRUE(node_->is_line_free(0.05, 0.05, 0.05, 0.95, 50, true));

  // マップ外への経路
  EXPECT_FALSE(node_->is_line_free(0.05, 0.05, 2.0, 2.0, 50, true));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
