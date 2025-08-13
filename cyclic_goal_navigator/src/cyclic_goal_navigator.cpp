#include "cyclic_goal_navigator/costmap_subscriber.hpp"
#include "cyclic_goal_navigator/goal_navigator.hpp"

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