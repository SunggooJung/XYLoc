cmake_minimum_required(VERSION 2.8.3)
project(ultrasound)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost COMPONENTS program_options REQUIRED)
#find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

add_definitions(-std=c++11)# -m64) #-Wall)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
   sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


link_directories(${Boost_LIBRARY_DIRS})

file(GLOB SOURCES
	src/ultrasound.cpp
)

#add_executable(stereo_vision_node src/stereo_vision_node.cpp)
add_executable(ultrasound ${SOURCES})


target_link_libraries(
	ultrasound
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
  
        ${Boost_LIBRARIES}
)


