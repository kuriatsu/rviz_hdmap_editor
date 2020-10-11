cmake_minimum_required(VERSION 2.8.3)
project(rviz_hdmap_editor)

add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON) # .ui to header

find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED
	Core
	Widgets
)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  tf
  visualization_msgs
  rviz
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES rviz_hdmap_editor_core
CATKIN_DEPENDS roscpp geometry_msgs interactive_markers visualization_msgs

#  DEPENDS system_lib
)

include_directories(
  include
	${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)

add_library(rviz_hdmap_editor_core lib/rviz_hdmap_editor_core.cpp)
add_dependencies(rviz_hdmap_editor_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_hdmap_editor_core
  ${catkin_LIBRARIES}
)

add_library(widget src/widget.cpp src/widget.ui)
add_dependencies(widget rviz_hdmap_editor_core)
target_link_libraries(widget rviz_hdmap_editor_core ${catkin_LIBRARIES} ${QT_LIBRARIES})
