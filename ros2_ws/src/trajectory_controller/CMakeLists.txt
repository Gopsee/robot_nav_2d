cmake_minimum_required(VERSION 3.8)
project(trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------
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

# -------------------------------------------------------------
# Generate Action Interfaces
# -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
)

# -------------------------------------------------------------
# Library (core controller logic)
# -------------------------------------------------------------
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

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -------------------------------------------------------------
# Node Executable
# -------------------------------------------------------------
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

# -------------------------------------------------------------
# Tests
# -------------------------------------------------------------
ament_add_gtest(test_${PROJECT_NAME}
  test/test_trajectory_controller.cpp
)

target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(test_${PROJECT_NAME}
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
)

# -------------------------------------------------------------
# Install
# -------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# -------------------------------------------------------------
# Export Dependencies
# -------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()


# cmake_minimum_required(VERSION 3.8)
# project(trajectory_controller)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # -------------------------------------------------------------
# # Dependencies
# # -------------------------------------------------------------
# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(action_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)

# # -------------------------------------------------------------
# # Generate Action Interfaces
# # -------------------------------------------------------------
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/FollowPath.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
# )

# # -------------------------------------------------------------
# # Executable
# # -------------------------------------------------------------
# add_executable(trajectory_controller_node
#   src/trajectory_controller_node.cpp
#   src/trajectory_controller.cpp
# )

# ament_target_dependencies(trajectory_controller_node
#   rclcpp
#   rclcpp_action
#   nav_msgs
#   geometry_msgs
#   tf2
#   tf2_geometry_msgs
#   visualization_msgs
# )

# # Link generated action interfaces (Jazzy-style)
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(trajectory_controller_node "${cpp_typesupport_target}")

# target_include_directories(trajectory_controller_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # -------------------------------------------------------------
# # Install
# # -------------------------------------------------------------
# install(TARGETS
#   trajectory_controller_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

# # -------------------------------------------------------------
# # Export Dependencies (for other packages like trajectory_generator)
# # -------------------------------------------------------------
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)

# ament_package()
