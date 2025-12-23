cmake_minimum_required(VERSION 3.8)
project(hyundai_steel_crane_automation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeadCR.msg"
  "msg/ACWL0001Body.msg"
  "msg/WLAC0001Body.msg"
  "srv/ACWL0001.srv"
  "srv/WLAC0001.srv"
  DEPENDENCIES builtin_interfaces
)

install(
  PROGRAMS
    src/acwl_client.py
    src/data_loader.py
    src/data_logger.py
    src/ftp_uploader.py
    src/socket_node.py
    src/protocol_parser.py
    src/inference_node.py
    src/sam2_wrapper.py
    src/utils.py
    src/visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(
  DIRECTORY 
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
