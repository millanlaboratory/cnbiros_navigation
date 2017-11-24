set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation
										cnbiros_core
										cnbiros_fusion)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include
					${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  SetStrengthSrv.srv
  SetDecaySrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	cnbiros_core
	cnbiros_fusion
)

add_library(cnbiros_navigation 
			src/TrigTools.cpp
			src/ForceField.cpp)
add_dependencies(cnbiros_navigation cnbiros_navigation_generate_messages_cpp)
target_link_libraries(cnbiros_navigation ${catkin_LIBRARIES})

