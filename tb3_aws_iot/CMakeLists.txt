cmake_minimum_required(VERSION 3.8)
project(tb3_aws_iot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# MQTT Client Library
find_package(PkgConfig REQUIRED)
pkg_check_modules(PAHO_MQTT_C QUIET libpaho-mqtt3as)
pkg_check_modules(PAHO_MQTT_CPP QUIET PahoMqttCpp)

# OpenSSL for TLS
find_package(OpenSSL REQUIRED)

# Try to find nlohmann_json, fallback to header-only if not found
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(NLOHMANN_JSON QUIET nlohmann_json)
  endif()
endif()

# Add executable
add_executable(aws_iot_publisher_node 
  src/aws_iot_publisher.cpp
  src/aws_iot_shadow_client.cpp
)
ament_target_dependencies(aws_iot_publisher_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# Link nlohmann_json if found
if(nlohmann_json_FOUND)
  target_link_libraries(aws_iot_publisher_node nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(aws_iot_publisher_node PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
  target_link_libraries(aws_iot_publisher_node ${NLOHMANN_JSON_LIBRARIES})
  target_compile_definitions(aws_iot_publisher_node PRIVATE ${NLOHMANN_JSON_CFLAGS_OTHER})
else()
  # Use header-only version - create simple JSON manually
  target_compile_definitions(aws_iot_publisher_node PRIVATE USE_SIMPLE_JSON)
endif()

# Link OpenSSL and include AWS IoT Shadow client
target_link_libraries(aws_iot_publisher_node OpenSSL::SSL OpenSSL::Crypto)
target_include_directories(aws_iot_publisher_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# We have our own MQTT implementation in aws_iot_shadow_client.cpp
# No need for external MQTT libraries

# Install targets
install(TARGETS
  aws_iot_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files  
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()