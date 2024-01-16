cmake_minimum_required(VERSION 3.8)
project(composable_nodes_test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        lifecycle_msgs
        message_filters
)

add_library(publisher_node SHARED src/publisher_node.cpp)
rclcpp_components_register_nodes(publisher_node "PublisherNode")
ament_target_dependencies(publisher_node ${dependencies})

install(
  TARGETS publisher_node
  LIBRARY DESTINATION lib
)

add_library(publisher_lifecycle_node SHARED src/publisher_lifecycle_node.cpp)
rclcpp_components_register_nodes(publisher_lifecycle_node "PublisherLifecycleNode")
ament_target_dependencies(publisher_lifecycle_node ${dependencies})

install(
  TARGETS publisher_lifecycle_node
  LIBRARY DESTINATION lib
)

add_library(subscriber_node SHARED src/subscriber_node.cpp)
rclcpp_components_register_nodes(subscriber_node "SubscriberNode")
ament_target_dependencies(subscriber_node ${dependencies})

install(
  TARGETS subscriber_node
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
