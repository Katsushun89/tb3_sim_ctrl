#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>
#include "cyclic_goal_navigator/goal_navigator.hpp"
#include "cyclic_goal_navigator/costmap_subscriber.hpp"

// テスト用のモックCostmapSubscriber
class MockCostmapSubscriber : public CostmapSubscriber
{
public:
  MockCostmapSubscriber()
  : CostmapSubscriber()
  {
    // テスト用のデータを設定
    test_cells_.clear();
    // 前方に候補点を配置
    for (double x = 1.0; x <= 5.0; x += 0.5) {
      test_cells_.push_back({x, 0.0});  // 前方直線上
      test_cells_.push_back({x, 0.5});  // 少し左
      test_cells_.push_back({x, -0.5}); // 少し右
    }
    test_frame_id_ = "test_frame";
  }

  std::vector<std::pair<double, double>> get_low_cost_cells_copy()
  {
    return test_cells_;
  }

  std::string get_costmap_frame_id()
  {
    return test_frame_id_;
  }

  bool is_goal_valid(double goal_x, double goal_y)
  {
    // テスト用: 原点から半径10m以内は有効とする
    double dist = std::sqrt(goal_x * goal_x + goal_y * goal_y);
    return dist <= 10.0;
  }

  bool is_goal_area_safe(double goal_x, double goal_y)
  {
    // テスト用: 原点から半径8m以内は安全とする
    double dist = std::sqrt(goal_x * goal_x + goal_y * goal_y);
    return dist <= 8.0;
  }

  bool is_line_free(
    double x0, double y0, double x1, double y1,
    int occ_threshold = 50, bool block_on_unknown = true)
  {
    (void)x0; (void)y0; (void)y1; (void)occ_threshold; (void)block_on_unknown;
    // テスト用: x=2.5に仮想的な壁があると仮定
    if (x1 > 2.5) {
      return false;  // 壁を通る
    }
    return true;  // それ以外は通行可能
  }

  // テスト用のデータ設定メソッド
  void set_test_cells(const std::vector<std::pair<double, double>> & cells)
  {
    test_cells_ = cells;
  }

private:
  std::vector<std::pair<double, double>> test_cells_;
  std::string test_frame_id_;
};

class GoalNavigatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    mock_costmap_ = std::make_shared<MockCostmapSubscriber>();
    navigator_ = std::make_shared<GoalNavigator>(mock_costmap_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<MockCostmapSubscriber> mock_costmap_;
  std::shared_ptr<GoalNavigator> navigator_;
};

// テスト: GoalNavigatorの基本的な初期化
TEST_F(GoalNavigatorTest, BasicInitialization)
{
  // GoalNavigatorが正常に初期化されることを確認
  EXPECT_NE(navigator_, nullptr);

  // モックCostmapSubscriberが正常に動作することを確認
  auto cells = mock_costmap_->get_low_cost_cells_copy();
  EXPECT_GT(cells.size(), 0u);

  auto frame_id = mock_costmap_->get_costmap_frame_id();
  EXPECT_EQ(frame_id, "test_frame");
}

// テスト: CostmapSubscriberのメソッドが正常に呼び出されることを確認
TEST_F(GoalNavigatorTest, CostmapSubscriberIntegration)
{
  // is_goal_validの動作確認
  EXPECT_TRUE(mock_costmap_->is_goal_valid(1.0, 1.0));   // 10m以内
  EXPECT_FALSE(mock_costmap_->is_goal_valid(15.0, 15.0)); // 10m超

  // is_goal_area_safeの動作確認
  EXPECT_TRUE(mock_costmap_->is_goal_area_safe(1.0, 1.0));   // 8m以内
  EXPECT_FALSE(mock_costmap_->is_goal_area_safe(10.0, 10.0)); // 8m超

  // is_line_freeの動作確認
  EXPECT_TRUE(mock_costmap_->is_line_free(0.0, 0.0, 2.0, 2.0));  // x=2.5未満
  EXPECT_FALSE(mock_costmap_->is_line_free(0.0, 0.0, 3.0, 3.0)); // x=2.5超
}

// テスト: セルデータの設定と取得
TEST_F(GoalNavigatorTest, CellDataManipulation)
{
  // カスタムセルデータを設定
  std::vector<std::pair<double, double>> custom_cells = {
    {1.0, 0.0},
    {2.0, 1.0},
    {3.0, -1.0}
  };

  mock_costmap_->set_test_cells(custom_cells);

  auto retrieved_cells = mock_costmap_->get_low_cost_cells_copy();
  EXPECT_EQ(retrieved_cells.size(), 3u);
  EXPECT_NEAR(retrieved_cells[0].first, 1.0, 0.001);
  EXPECT_NEAR(retrieved_cells[0].second, 0.0, 0.001);
  EXPECT_NEAR(retrieved_cells[1].first, 2.0, 0.001);
  EXPECT_NEAR(retrieved_cells[1].second, 1.0, 0.001);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
