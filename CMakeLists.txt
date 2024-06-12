cmake_minimum_required(VERSION 3.5)
project(my_rviz2_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

# my_rviz2_plugin ライブラリの設定
add_library(my_rviz2_plugin SHARED
  src/my_rviz2_plugin.cpp
)

ament_target_dependencies(my_rviz2_plugin
  rclcpp
  std_msgs
  rviz_common
  Qt5
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS my_rviz2_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# signal_generator 実行ファイルの設定
add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

install(TARGETS
  signal_generator
  DESTINATION lib/${PROJECT_NAME})

ament_package()