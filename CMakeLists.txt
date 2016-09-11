cmake_minimum_required(VERSION 2.8)
project(kinect_utlities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g" )

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#==============================================================================
# Dependencies
#==============================================================================
# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIR})

include_directories(include)
#==============================================================================
# Application
#==============================================================================
add_executable(kinect_utlities src/main.cpp)
target_link_libraries(kinect_utlities ${freenect2_LIBRARIES} ${OpenCV_LIBS})
