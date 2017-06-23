set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
										geometry_msgs 
										sensor_msgs
			 							grid_map_core 
										grid_map_ros 
										grid_map_msgs 
										grid_map_cv 
										cv_bridge
										cnbiros_core)

# Eigen3
find_package(PkgConfig)
pkg_search_module(EIGEN3 REQUIRED eigen3>=3.2.9)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include
					${catkin_INCLUDE_DIRS} 
					${EIGEN3_INCLUDE_DIRS})
add_service_files(
  FILES
  ResetGridService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS 
	include
	${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	cnbiros_core
	grid_map_ros
)

add_library(cnbiros_navigation 
			src/SimpleFusion.cpp
			src/FusionGrid.cpp
			src/ForceField.cpp)
target_link_libraries(cnbiros_navigation ${catkin_LIBRARIES})

add_executable(example_fusion examples/example_fusion.cpp)
target_link_libraries(example_fusion cnbiros_navigation)

