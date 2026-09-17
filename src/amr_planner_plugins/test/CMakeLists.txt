ament_add_gtest(test_amr_planner_plugins
  test_main.cpp
  test_grid_search_planners.cpp
  test_rrt_planner.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_include_directories(test_amr_planner_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(test_amr_planner_plugins ${dependencies})
target_link_libraries(test_amr_planner_plugins ${library_name})
