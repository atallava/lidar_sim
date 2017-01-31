# Version required.
cmake_minimum_required (VERSION 2.8)

project (LIDAR_SIM)

set (CMAKE_BUILD_TYPE "Debug")

set( LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib" CACHE PATH
    "Target directory for all libraries." )
set( EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
    "Target directory for all executables." )

# Threading
find_package( Threads REQUIRED )
list( APPEND LIDAR_SIM_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

# Boost
find_package( Boost COMPONENTS system thread log REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND LIDAR_SIM_LIBRARIES ${Boost_LIBRARIES} )
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_thread" ) # Force linking against thread
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_system" ) # Force linking against system

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
list( APPEND LIDAR_SIM_LIBRARIES ${VTK_LIBRARIES} )

# PCL
find_package( PCL 1.4 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})
list( APPEND LIDAR_SIM_LIBRARIES ${PCL_LIBRARIES})

# ALGLIB
set( ALGLIB_INCLUDE_DIR "/usr0/home/atallav1/alglib-3.10.0.cpp/src" )
include_directories( ${ALGLIB_INCLUDE_DIR} )
set( ALGLIB_LIBRARY "/usr0/home/atallav1/alglib-3.10.0.cpp/lib/libalglib.so" )
set( ALGLIB_LIBRARY_DIR "/usr0/home/atallav1/alglib-3.10.0.cpp/lib/" )
link_directories( ${ALGLIB_LIBRARY_DIR} )
list( APPEND LIDAR_SIM_LIBRARIES ${ALGLIB_LIBRARY} )

# EIGENMVN
set( EIGENMVN_INCLUDE_DIR "/usr0/home/atallav1/eigenmvn" )
include_directories( ${EIGENMVN_INCLUDE_DIR} )

# LIBGP
set( LIBGP_INCLUDE_DIR "/usr0/home/atallav1/libgp/include" )
include_directories( ${LIBGP_INCLUDE_DIR} )
set( LIBGP_LIBRARY "/usr0/home/atallav1/libgp/build/libgp.a" )
set( LIBGP_LIBRARY_DIR "/usr0/home/atallav1/libgp/build" )
link_directories( ${LIBGP_LIBRARY_DIR} )
list( APPEND LIDAR_SIM_LIBRARIES ${LIBGP_LIBRARY} )

# FLANN
# commenting out because flann headers seem to already be in /usr/include
# set( FLANN_INCLUDE_DIR "/usr0/home/atallav1/" )
# include_directories( ${FLANN_INCLUDE_DIR} )

# CGAL
find_package( CGAL)
include( ${CGAL_USE_FILE} ) 
include( CGAL_CreateSingleSourceCGALProgram ) 
# message( STATUS "CGAL FOUND: " ${CGAL_FOUND} )
# message( FATAL_ERROR )
  
# c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_definitions( -Wall -std=c++0x )

# This project's headers
set( LIDAR_SIM_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )
include_directories(${LIDAR_SIM_INCLUDE_DIRECTORIES})

# Add files here to compile them
add_library(lidar_sim SHARED 	

  src/Visualizer.cpp
  src/Test.cpp
  
  src/PoseServer.cpp
  src/FrameTimeServer.cpp
  src/DataProcessingUtils.cpp

  src/EllipsoidVtkActorServer.cpp
  src/PointsVtkActorServer.cpp
  src/LineVtkActorServer.cpp
  src/VizUtils.cpp
  src/EllipsoidModelUtils.cpp
  src/MathUtils.cpp
  src/RangeDataVizer.cpp 
  src/ClusteringUtils.cpp
  src/LaserCalibParams.cpp
  src/PoseUtils.cpp
  src/EllipsoidModelSim.cpp
  src/SectionLoader.cpp
  src/LaserUtils.cpp
  src/GroundModeler.cpp
  )
 
# Executables

# tests
file( GLOB TESTS_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} tests/*.cpp )
MESSAGE( STATUS "tests sources: " ${TESTS_SOURCES} )
foreach( testsourcefile ${TESTS_SOURCES} )
  string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
  string( REPLACE "tests/" "" testname ${testnamewithdir})
  add_executable( ${testname} ${testsourcefile} )
  target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
endforeach( testsourcefile ${TESTS_SOURCES} )

# analyze taylor data
file( GLOB ANALYZE_TAYLOR_DATA_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} analyze_taylor_data/*.cpp )
foreach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )
  string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
  string( REPLACE "analyze_taylor_data/" "" testname ${testnamewithdir})
  add_executable( ${testname} ${testsourcefile} )
  target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
endforeach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )
 
# vane sim
file( GLOB VANE_SIM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} vane_sim/*.cpp )
foreach( testsourcefile ${VANE_SIM_SOURCES} )
  string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
  string( REPLACE "vane_sim/" "" testname ${testnamewithdir})
   
  add_executable( ${testname} ${testsourcefile} )
  target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
endforeach( testsourcefile ${VANE_SIM_SOURCES} )

 