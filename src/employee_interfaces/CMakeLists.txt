cmake_minimum_required(VERSION 3.8)
project(resident_recognition_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지와 서비스 정의
set(msg_files
  "msg/DetectedObject.msg"
)

set(srv_files
  "srv/CustomService.srv"
)

# ROS 2 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

