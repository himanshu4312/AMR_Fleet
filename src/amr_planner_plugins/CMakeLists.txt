cmake_minimum_required(VERSION 3.8)
project(amr_planner_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(library_name amr_planner_plugins)
set(dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
                  nav_msgs geometry_msgs tf2_ros pluginlib)

add_library(${library_name} SHARED
  src/grid_search_planner.cpp
  src/astar_planner.cpp
  src/dijkstra_planner.cpp
  src/rrt_planner.cpp
)
ament_target_dependencies(${library_name} ${dependencies})
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES global_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  add_executable(manual_test_planners test/manual_test_planners.cpp)
  target_include_directories(manual_test_planners PRIVATE include)
  ament_target_dependencies(manual_test_planners ${dependencies})
  target_link_libraries(manual_test_planners ${library_name})

  add_executable(benchmark_planners test/benchmark_planners.cpp)
  target_include_directories(benchmark_planners PRIVATE include test)
  ament_target_dependencies(benchmark_planners ${dependencies})
  target_link_libraries(benchmark_planners ${library_name})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()