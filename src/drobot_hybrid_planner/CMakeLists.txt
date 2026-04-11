cmake_minimum_required(VERSION 3.8)
project(drobot_hybrid_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# TODO: 플래너 플러그인 라이브러리 추가
# add_library(hybrid_rrt_star_planner SHARED
#   src/hybrid_rrt_star_planner.cpp
# )
# target_include_directories(hybrid_rrt_star_planner PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(hybrid_rrt_star_planner
#   rclcpp nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib
# )
# pluginlib_export_plugin_description_file(nav2_core hybrid_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install(TARGETS hybrid_rrt_star_planner
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
