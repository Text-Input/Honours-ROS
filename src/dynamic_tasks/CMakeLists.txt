cmake_minimum_required(VERSION 3.8)
project(dynamic_tasks)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamic_interfaces REQUIRED)
find_package(cxxopts REQUIRED)

add_executable(dynamic_tasks src/main.cpp
        src/agent.cpp
        include/vec.h
        src/world_info_provider.cpp
        include/common.h
        include/world_info_provider.h
        src/task_allocator.cpp
        include/task_allocator.h
        src/dynamic_algs/simple.cpp)
target_include_directories(dynamic_tasks PUBLIC include/)
ament_target_dependencies(dynamic_tasks rclcpp std_msgs geometry_msgs dynamic_interfaces cxxopts)

install(TARGETS dynamic_tasks DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
