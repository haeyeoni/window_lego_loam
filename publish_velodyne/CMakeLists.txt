cmake_minimum_required(VERSION 3.0.2)
project(publish_velodyne)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_DEBUG ON)
# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )


# Find Packages
set(PCL_ROOT "C:/Program\ Files/PCL\ 1.8.1")
set(PCL_INCLUDE_DIRS "C:/Program\ Files/PCL\ 1.8.1/include")
set(PCL_LIBRARIES "C:/Program\ Files/PCL\ 1.8.1/lib")

find_package( PCL REQUIRED QUIET)
#find_package(VTK REQUIRED COMPONENTS vtkCommonCore vtkCommonDataModel)

FIND_PACKAGE(Boost QUIET)

IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

find_package(catkin 
REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros nodelet
 DEPENDS PCL
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}

    ${Boost_INCLUDE_DIRS}
)
add_definitions( ${PCL_DEFINITIONS} )

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()