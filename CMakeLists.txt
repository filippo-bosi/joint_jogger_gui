cmake_minimum_required(VERSION 3.14)
project(joint_jogger_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# Qt
find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(QT_LIBS Qt5::Widgets)

# Enable Qt MOC so Q_OBJECT works
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# RCC/ UIC/ MOC not needed here; pure code widgets

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/joint_jogger_panel.cpp
  include/joint_jogger_gui/joint_jogger_panel.hpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  QT_NO_KEYWORDS
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  std_msgs
)

# Export RViz plugin symbol
pluginlib_export_plugin_description_file(rviz_common resource/joint_jogger_gui_plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES resource/joint_jogger_gui_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/resource)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()