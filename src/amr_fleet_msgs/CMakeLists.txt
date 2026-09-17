cmake_minimum_required(VERSION 3.8)
project(amr_fleet_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# No .msg/.srv files yet -- this list will be populated in a later step.
set(msg_files
)

# Skipped for now since msg_files is empty -- rosidl_generate_interfaces()
# errors out if called with no interface files. This will start generating
# interfaces automatically once .msg/.srv files are added above.
if(msg_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
