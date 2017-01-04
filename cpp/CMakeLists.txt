# Version required.
cmake_minimum_required (VERSION 2.8)

project (LIDAR_SIM)

set (CMAKE_BUILD_TYPE "Debug")

set( LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib" CACHE PATH
    "Target directory for all libraries." )
set( EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
    "Target directory for all executables." )

# Threading and Boost utilities
find_package( Threads REQUIRED )
list( APPEND LIDAR_SIM_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

find_package( Boost COMPONENTS system thread log REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND LIDAR_SIM_LIBRARIES ${Boost_LIBRARIES} )
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_thread" ) # Force linking against thread
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_system" ) # Force linking against system

# PCL for visualizing
find_package( PCL 1.4 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})
list( APPEND LIDAR_SIM_LIBRARIES ${PCL_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_definitions( -Wall -std=c++0x )

# This project's headers
set( LIDAR_SIM_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )
include_directories(${LIDAR_SIM_INCLUDE_DIRECTORIES})

# Add files here to compile them
add_library(lidar_sim SHARED 	

  src/Visualizer.cpp
  src/Test.cpp
  src/Utils.cpp
  src/PoseServer.cpp
  )

# Add files here to make executables out of them
add_executable(run_tests src/run_tests.cpp)
target_link_libraries(run_tests lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(split_packets src/split_packets.cpp)
target_link_libraries(split_packets lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(transform_sections_to_world src/transform_sections_to_world.cpp)
target_link_libraries(transform_sections_to_world lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(subsample_sections src/subsample_sections.cpp)
target_link_libraries(subsample_sections lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(subsample_georef src/subsample_georef.cpp)
target_link_libraries(subsample_georef lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(prepend_pcd_header src/prepend_pcd_header.cpp)
target_link_libraries(prepend_pcd_header lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(extract_section_packet_ids src/extract_section_packet_ids.cpp)
target_link_libraries(extract_section_packet_ids lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(extract_from_georef src/extract_from_georef.cpp)
target_link_libraries(extract_from_georef lidar_sim ${LIDAR_SIM_LIBRARIES})

add_executable(extract_scans_from_rip src/extract_scans_from_rip.cpp)
target_link_libraries(extract_scans_from_rip lidar_sim ${LIDAR_SIM_LIBRARIES})
