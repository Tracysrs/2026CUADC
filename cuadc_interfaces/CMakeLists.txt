cmake_minimum_required(VERSION 3.8)
project(cuadc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 接口清单与《接口契约.md》同步维护：新增/变更消息必须先改契约文档再改这里。
# SafetyStatus（安全联锁，M4）与更多判读字段后续追加到本清单。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReconMarker.msg"
  "msg/ReconClassification.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
