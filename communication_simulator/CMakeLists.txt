cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


find_package( CGAL COMPONENTS Core PATHS /opt/cgal)

if(NOT CGAL_FOUND)
  MESSAGE(FATAL_ERROR "Could not find cgal")
endif (NOT CGAL_FOUND)

include( ${CGAL_USE_FILE} )

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}BasicRadioModel src/commsSimulatorBase.cc src/commsSimulatorBasicRadioModel.cc src/SignalPathSimulator.cc)

if ( CGAL_AUTO_LINK_ENABLED )
  target_link_libraries(${PROJECT_NAME}BasicRadioModel gsl gslcblas  ${CGL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(${PROJECT_NAME}BasicRadioModel gsl gslcblas ${CGAL_LIBRARIES} ${CGL_3RD_PARTY_LIBRARIES} )
endif()

rosbuild_add_library(${PROJECT_NAME}BasicRangeModel src/commsSimulatorBase.cc src/commsSimulatorBasicRangeModel.cc src/SignalPathSimulator.cc)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(sim_basic_radio_node src/sim_basic_radio_node.cc)

if ( CGAL_AUTO_LINK_ENABLED )
  target_link_libraries(sim_basic_radio_node ${PROJECT_NAME}BasicRadioModel gsl gslcblas  ${CGL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(sim_basic_radio_node ${PROJECT_NAME}BasicRadioModel gsl gslcblas ${CGAL_LIBRARIES} ${CGL_3RD_PARTY_LIBRARIES} )
endif()

rosbuild_add_executable(testclass src/testclass.cc)
if ( CGAL_AUTO_LINK_ENABLED )
  target_link_libraries(testclass ${PROJECT_NAME}BasicRadioModel gsl gslcblas  ${CGL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(testclass ${PROJECT_NAME}BasicRadioModel gsl gslcblas ${CGAL_LIBRARIES} ${CGL_3RD_PARTY_LIBRARIES} )
endif()

rosbuild_add_executable(test_signal_path_simulator src/test_signal_path_simulator.cc src/SignalPathSimulator)

if ( CGAL_AUTO_LINK_ENABLED )
  target_link_libraries(test_signal_path_simulator gsl gslcblas  ${CGL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(test_signal_path_simulator gsl gslcblas ${CGAL_LIBRARIES} ${CGL_3RD_PARTY_LIBRARIES} )
endif()


