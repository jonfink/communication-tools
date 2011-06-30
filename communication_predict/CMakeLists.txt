cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE Debug)
find_package(CGAL QUIET COMPONENTS Core PATHS /opt/cgal)

if (NOT CGAL_FOUND)
  message(FATAL_ERROR "This program requires the CGAL library, and will not be compiled.")
endif(NOT CGAL_FOUND)

include( ${CGAL_USE_FILE} )

rosbuild_add_library(CommunicationPredict src/CommunicationPredict.cc)
target_link_libraries(CommunicationPredict ${CGAL_LIBRARIES})