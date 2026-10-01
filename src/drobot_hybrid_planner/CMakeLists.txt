cmake_minimum_required(VERSION 3.8)
project(drobot_hybrid_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Nav2 Jazzy는 C++17을 쓴다
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(drobot_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  std_msgs
  pluginlib
  tf2_ros
  drobot_msgs
)

add_library(hybrid_astar_planner SHARED
  src/energy_model.cpp
  src/state_space.cpp
  src/hybrid_astar_planner.cpp
)

target_include_directories(hybrid_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hybrid_astar_planner ${dependencies})

# nav2_core 플러그인으로 등록
pluginlib_export_plugin_description_file(nav2_core hybrid_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Python↔C++ 동등성 테스트.
  # 기댓값(test/energy_fixtures.hpp)은 benchmark/gen_cpp_fixtures.py 가
  # Python 벤치마크에서 뽑아 둔 것이다. energy_params.yaml 을 바꾸면
  # 그 스크립트를 다시 돌려야 한다.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_energy_model test/test_energy_model.cpp)
  target_include_directories(test_energy_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_energy_model hybrid_astar_planner)
  ament_target_dependencies(test_energy_model ${dependencies})

  # 상태공간 전이 테스트.
  # 특히 '비행으로 장애물을 넘을 수 있는가' — 3D 고도 처리 방식이
  # 상승각 제약과 충돌하는지 잡는다. 파일 상단 주석 참고.
  ament_add_gtest(test_state_space test/test_state_space.cpp)
  target_include_directories(test_state_space PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_state_space hybrid_astar_planner)
  ament_target_dependencies(test_state_space ${dependencies})

  # 아래 두 테스트는 노드를 만든다. 떠 있는 시뮬레이션과 DDS 로 섞이지 않게
  # ROS_DOMAIN_ID 를 격리해 돌린다 (특히 첫 번째는 노드 이름이 planner_server 다).
  find_package(ament_cmake_ros REQUIRED)

  # nav2 파라미터 파일 -> planner_server -> EnergyModel 전달 경로.
  # 파일 구조가 아니라 '값이 노드에 도달하는가'를 본다 (파일 상단 주석 참고).
  ament_add_ros_isolated_gtest(test_nav2_params_file test/test_nav2_params_file.cpp)
  target_include_directories(test_nav2_params_file PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(test_nav2_params_file PRIVATE
    DROBOT_BRINGUP_NAV_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../drobot_bringup/config/navigation"
  )
  target_link_libraries(test_nav2_params_file hybrid_astar_planner)
  ament_target_dependencies(test_nav2_params_file ${dependencies})

  # cost <-> 높이 계약이 호스트 inflation 설정에 오염되는 정도 (측정/특성화).
  # 실제 nav2_costmap_2d::InflationLayer 를 쓴다. 결과 정리: COST_HEIGHT_CONTRACT.md
  ament_add_ros_isolated_gtest(test_costmap_terrain_contract
    test/test_costmap_terrain_contract.cpp)
  target_include_directories(test_costmap_terrain_contract PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_costmap_terrain_contract hybrid_astar_planner)
  ament_target_dependencies(test_costmap_terrain_contract ${dependencies})
endif()

install(TARGETS hybrid_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(FILES hybrid_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hybrid_astar_planner)
ament_export_dependencies(${dependencies})

ament_package()
