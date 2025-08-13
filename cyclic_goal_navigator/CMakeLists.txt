cmake_minimum_required(VERSION 3.8)
project(cyclic_goal_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
)

# 実行可能ファイルの作成
add_executable(cyclic_goal_navigator_node
  src/cyclic_goal_navigator.cpp
)

# インクルードパスの設定
target_include_directories(cyclic_goal_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 依存関係の設定
ament_target_dependencies(cyclic_goal_navigator_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  visualization_msgs
)

# インストール設定
install(TARGETS
  cyclic_goal_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ヘッダーファイルのインストール
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
