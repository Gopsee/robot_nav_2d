cmake_minimum_required(VERSION 3.8)
project(trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Generate action messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
)

# Library
add_library(${PROJECT_NAME}_lib
  src/trajectory_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

# Link ROS2 Type Support for the generated action
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# Ensure library waits for action messages to be generated
add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable Node
add_executable(${PROJECT_NAME}_node
  src/trajectory_controller_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install library to standard lib directory
install(TARGETS
  ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
)

# Install node executable to ros2 run path
install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)


# Unit Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_trajectory_controller.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}_lib
    )
    ament_target_dependencies(test_${PROJECT_NAME}
      rclcpp
      rclcpp_action
      geometry_msgs
      nav_msgs
    )
  endif()
endif()

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
