cmake_minimum_required(VERSION 3.8)
project(robit_navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 계산 부분 (ROS2 없이도 쓸 수 있는 라이브러리)
add_library(nav_core src/nav_core.cpp src/astar.cpp)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 내비게이션 노드 (내 담당)
add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node nav_core)
ament_target_dependencies(navigation_node
  rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)

# 가짜 로봇 노드 (연습용)
add_executable(fake_robot_node src/fake_robot_node.cpp)
ament_target_dependencies(fake_robot_node rclcpp geometry_msgs visualization_msgs)

# 계산 확인용 테스트 (ROS2 없이 실행 가능)
add_executable(test_core test/test_core.cpp)
target_link_libraries(test_core nav_core)

install(TARGETS nav_core navigation_node fake_robot_node test_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
