cmake_minimum_required(VERSION 3.8)
project(cuadc_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(cuadc_mission_node src/mission_node.cpp)
target_include_directories(cuadc_mission_node PRIVATE include)
ament_target_dependencies(cuadc_mission_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  mavros_msgs
)

install(TARGETS cuadc_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 投放决策核心单元测试（无 ROS 依赖，colcon test 即跑；也可手动编译：
#   g++ -std=c++17 -I include test/test_drop_logic.cpp -o t && ./t）
if(BUILD_TESTING)
  add_executable(test_drop_logic test/test_drop_logic.cpp)
  target_include_directories(test_drop_logic PRIVATE include)
  add_test(NAME test_drop_logic COMMAND test_drop_logic)
  # 航线构建/坐标换算纯函数单测（"横着搜"回归防线，09-12）
  add_executable(test_route_logic test/test_route_logic.cpp)
  target_include_directories(test_route_logic PRIVATE include)
  add_test(NAME test_route_logic COMMAND test_route_logic)
endif()

ament_package()
