cmake_minimum_required(VERSION 2.8.3)
project(rviz_hdmap_builder)

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
  pcl_ros
  rviz
)

catkin_package(
#	INCLUDE_DIRS include
#  LIBRARIES extract_roi
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
#	include
	${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rviz_adas_map_builder.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_library(widget src/widget.cpp src/widget.ui)
target_link_libraries(widget ${catkin_LIBRARIES} ${QT_LIBRARIES})
