cmake_minimum_required(VERSION 3.8)
project(path_smoothing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Library
add_library(${PROJECT_NAME}_lib
  src/path_smoothing.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Node Executable
add_executable(${PROJECT_NAME}_node
  src/path_smoothing_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install Targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Headers
install(DIRECTORY include/
  DESTINATION include
)

# Testing (Optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(path_smoothing)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(ament_cmake_gtest REQUIRED)

# include_directories(include)

# add_library(${PROJECT_NAME}_lib
#   src/path_smoothing.cpp
# )

# target_include_directories(${PROJECT_NAME}_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(${PROJECT_NAME}_lib
#   rclcpp
#   nav_msgs
#   geometry_msgs
# )

# add_executable(path_smoothing_node
#   src/path_smoothing_node.cpp
# )

# # target_link_libraries(path_smoothing_node
# #   path_smoothingSHARED
# # )

# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   nav_msgs
#   geometry_msgs
# )

# # install(TARGETS path_smoothing
# #   LIBRARY DESTINATION lib/${PROJECT_NAME}/
# # )

# install(TARGETS path_smoothing_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )


# # install(TARGETS
# #   path_smoothing
# #   path_smoothing_node
# #   DESTINATION lib/${PROJECT_NAME}
# # )

# install(DIRECTORY include/
#   DESTINATION include
# )

# # Unit Tests
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   # ament_add_gtest(test_path_smoothing_node test/test_path_smoothing_node.cpp)
#   # if(TARGET test_path_smoothing_node)
#   #   target_link_libraries(test_path_smoothing_node path_smoothing)
#   #   ament_target_dependencies(test_path_smoothing_node
#   #     rclcpp
#   #     nav_msgs
#   #     geometry_msgs
#   #   )
#   #   install(TARGETS test_path_smoothing_node
#   #     DESTINATION lib/${PROJECT_NAME}
#   #   )
#   # endif()
# endif()

# ament_package()
