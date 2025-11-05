cmake_minimum_required(VERSION 3.8)
project(path_smoothing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(include)

# Library Target
add_library(path_smoothing SHARED
  src/path_smoothing.cpp
)

target_include_directories(path_smoothing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_smoothing
  rclcpp
  nav_msgs
  geometry_msgs
)

# Node Executable
add_executable(path_smoothing_node
  src/path_smoothing_node.cpp
)

target_link_libraries(path_smoothing_node
  path_smoothing
)

ament_target_dependencies(path_smoothing_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install
install(TARGETS
  path_smoothing
  path_smoothing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Unit Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_path_smoothing_node test/test_path_smoothing_node.cpp)
  if(TARGET test_path_smoothing_node)
    target_link_libraries(test_path_smoothing_node path_smoothing)
    ament_target_dependencies(test_path_smoothing_node
      rclcpp
      nav_msgs
      geometry_msgs
    )
    install(TARGETS test_path_smoothing_node
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endif()

ament_package()
