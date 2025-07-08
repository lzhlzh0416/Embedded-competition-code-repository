cmake_minimum_required(VERSION 3.5)
project(can_bridge_cpp)

# 编译选项
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# 头文件目录
include_directories(
  include
)

# 添加可执行文件
add_executable(can_sender_cpp
  src/can_sender.cpp
  src/socketcan.cpp
)

# 链接依赖
ament_target_dependencies(can_sender_cpp
  rclcpp
  can_msgs
  nav_msgs
)

# 安装目标
install(TARGETS
  can_sender_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
