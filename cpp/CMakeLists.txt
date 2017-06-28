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
find_package( Boost COMPONENTS system thread log regex REQUIRED )
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

  src/Test.cpp
  src/Visualizer.cpp
  
  src/PoseServer.cpp
  src/FrameTimeServer.cpp
  src/DataProcessingUtils.cpp

  src/EllipsoidVtkActorServer.cpp
  src/PointsVtkActorServer.cpp
  src/LineVtkActorServer.cpp
  src/TrianglesVtkActorServer.cpp
  src/VizUtils.cpp
  src/RangeDataVizer.cpp 

  src/MathUtils.cpp
  src/LaserCalibParams.cpp
  src/LaserUtils.cpp
  src/PoseUtils.cpp
  src/ClusteringUtils.cpp
  src/ModelingUtils.cpp

  src/SectionLoader.cpp
  src/GeometricSegmenter.cpp
  src/TriangleModeler.cpp
  src/EllipsoidModeler.cpp
  src/EllipsoidModelSim.cpp
  src/TriangleModelSim.cpp
  src/SectionModelSim.cpp
  src/SimDetail.cpp

  src/NearestNeighborSim.cpp

  src/PtsError.cpp

  src/GroundTrianglesGenerator.cpp
  src/OrientedBox.cpp
  src/SceneObjectServer.cpp
  src/MeshModelSim.cpp
  )
 
####################################################################################################

# Executables

####################################################################################################

# tests
# file( GLOB TESTS_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} tests/*.cpp )
# MESSAGE( STATUS "tests sources: " ${TESTS_SOURCES} )
# foreach( testsourcefile ${TESTS_SOURCES} )
#   string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
#   string( REPLACE "tests/" "" testname ${testnamewithdir})
#   add_executable( ${testname} ${testsourcefile} )
#   target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
# endforeach( testsourcefile ${TESTS_SOURCES} )

# add_executable(run_tests tests/run_tests.cpp)
#   target_link_libraries(run_tests lidar_sim ${LIDAR_SIM_LIBRARIES} )


####################################################################################################

# analyze taylor data
# file( GLOB ANALYZE_TAYLOR_DATA_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} analyze_taylor_data/*.cpp )
# foreach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )
#   string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
#   string( REPLACE "analyze_taylor_data/" "" testname ${testnamewithdir})
#   add_executable( ${testname} ${testsourcefile} )
#   target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
# endforeach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )
  
## pattern

# # add_executable(pattern analyze_taylor_data/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(subsample_section_pts analyze_taylor_data/subsample_section_pts.cpp)
#   target_link_libraries(subsample_section_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(transform_sections_to_world analyze_taylor_data/transform_sections_to_world.cpp)
#   target_link_libraries(transform_sections_to_world lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

# vtk examples
# file( GLOB ANALYZE_TAYLOR_DATA_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} vtk_examples/*.cpp )
# foreach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )
#   string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
#   string( REPLACE "vtk_examples/" "" testname ${testnamewithdir})
#   add_executable( ${testname} ${testsourcefile} )
#   target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
# endforeach( testsourcefile ${ANALYZE_TAYLOR_DATA_SOURCES} )

####################################################################################################
 
# hybrid_geometric sim
# file( GLOB HYBRID_GEOMETRIC_SIM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} hybrid_geometric_sim/*.cpp )
# foreach( testsourcefile ${HYBRID_GEOMETRIC_SIM_SOURCES} )
#   string( REPLACE ".cpp" "" testnamewithdir ${testsourcefile} )
#   string( REPLACE "hybrid_geometric_sim/" "" testname ${testnamewithdir})
   
#   add_executable( ${testname} ${testsourcefile} )
#   target_link_libraries( ${testname} lidar_sim ${LIDAR_SIM_LIBRARIES} )
# endforeach( testsourcefile ${HYBRID_GEOMETRIC_SIM_SOURCES} )
 
## pattern

# # add_executable(pattern dir/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

## tests

# add_executable(test_flann tests/test_flann.cpp)
#   target_link_libraries(test_flann lidar_sim ${LIDAR_SIM_LIBRARIES} )

## data processing

# add_executable(sieve_section hybrid_geometric_sim/sieve_section.cpp)
#   target_link_libraries(sieve_section lidar_sim ${LIDAR_SIM_LIBRARIES} )

## segmentation

# add_executable(segmentation_trial hybrid_geometric_sim/segmentation_trial.cpp)
#   target_link_libraries(segmentation_trial lidar_sim ${LIDAR_SIM_LIBRARIES} )

## modeling

# add_executable(gen_section_imu_posn_nodes hybrid_geometric_sim/gen_section_imu_posn_nodes.cpp)
#   target_link_libraries(gen_section_imu_posn_nodes lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_blocks_ground hybrid_geometric_sim/build_blocks_ground.cpp)
#   target_link_libraries(build_blocks_ground lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_blocks_non_ground hybrid_geometric_sim/build_blocks_non_ground.cpp)
#   target_link_libraries(build_blocks_non_ground lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_models_ground_blocks hybrid_geometric_sim/build_models_ground_blocks.cpp)
#   target_link_libraries(build_models_ground_blocks lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_models_non_ground_blocks hybrid_geometric_sim/build_models_non_ground_blocks.cpp)
#   target_link_libraries(build_models_non_ground_blocks lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(debug_hit_prob hybrid_geometric_sim/debug_hit_prob.cpp)
#   target_link_libraries(debug_hit_prob lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(patch_hit_prob_non_ground_blocks hybrid_geometric_sim/patch_hit_prob_non_ground_blocks.cpp)
#   target_link_libraries(patch_hit_prob_non_ground_blocks lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(patch_hit_prob_ground_blocks hybrid_geometric_sim/patch_hit_prob_ground_blocks.cpp)
#   target_link_libraries(patch_hit_prob_ground_blocks lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(analyze_clustering hybrid_geometric_sim/analyze_clustering.cpp)
#   target_link_libraries(analyze_clustering lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(analyze_mesh hybrid_geometric_sim/analyze_mesh.cpp)
#   target_link_libraries(analyze_mesh lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(get_non_ground_from_ground_pts hybrid_geometric_sim/get_non_ground_from_ground_pts.cpp)
#   target_link_libraries(get_non_ground_from_ground_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_primitives scene_sim/build_primitives.cpp)
#   target_link_libraries(build_primitives lidar_sim ${LIDAR_SIM_LIBRARIES} )

## state mapping

# add_executable(gen_ground_triangles hybrid_geometric_sim/gen_ground_triangles.cpp)
#   target_link_libraries(gen_ground_triangles lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_sim_scene hybrid_geometric_sim/build_sim_scene.cpp)
#   target_link_libraries(build_sim_scene lidar_sim ${LIDAR_SIM_LIBRARIES} )

## hg sim

# add_executable(poses_section_models_sim hybrid_geometric_sim/poses_section_models_sim.cpp)
#   target_link_libraries(poses_section_models_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )
 
# add_executable(section_ray_hg_sim hybrid_geometric_sim/section_ray_hg_sim.cpp)
#   target_link_libraries(section_ray_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_slice_hg_sim hybrid_geometric_sim/section_slice_hg_sim.cpp)
#   target_link_libraries(section_slice_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_hg_sim hybrid_geometric_sim/section_hg_sim.cpp)
#   target_link_libraries(section_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## nbr sim

# add_executable(poses_nbr_sim hybrid_geometric_sim/poses_nbr_sim.cpp)
#   target_link_libraries(poses_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_slice_nbr_sim hybrid_geometric_sim/section_slice_nbr_sim.cpp)
#   target_link_libraries(section_slice_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_nbr_sim hybrid_geometric_sim/section_nbr_sim.cpp)
#   target_link_libraries(section_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## scene sim

# hg sim

add_executable(section_slice_hg_scene_sim scene_sim/section_slice_hg_scene_sim.cpp)
  target_link_libraries(section_slice_hg_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# mesh model sim

# add_executable(test_mesh_model_sim scene_sim/test_mesh_model_sim.cpp)
#   target_link_libraries(test_mesh_model_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

add_executable(section_slice_mm_scene_sim scene_sim/section_slice_mm_scene_sim.cpp)
  target_link_libraries(section_slice_mm_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

add_executable(viz_scene_mesh_models scene_sim/viz_scene_mesh_models.cpp)
  target_link_libraries(viz_scene_mesh_models lidar_sim ${LIDAR_SIM_LIBRARIES} )

# nbr sim

add_executable(section_slice_nbr_scene_sim scene_sim/section_slice_nbr_scene_sim.cpp)
  target_link_libraries(section_slice_nbr_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## viz

# add_executable(write_section_models_fig hybrid_geometric_sim/write_section_models_fig.cpp)
#   target_link_libraries(write_section_models_fig lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(viz_section_models hybrid_geometric_sim/viz_section_models.cpp)
#   target_link_libraries(viz_section_models lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(viz_ellipsoid_models hybrid_geometric_sim/viz_ellipsoid_models.cpp)
#   target_link_libraries(viz_ellipsoid_models lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(viz_triangle_models hybrid_geometric_sim/viz_triangle_models.cpp)
#   target_link_libraries(viz_triangle_models lidar_sim ${LIDAR_SIM_LIBRARIES} )

## eval

# add_executable(eval_sims hybrid_geometric_sim/eval_sims.cpp)
#   target_link_libraries(eval_sims lidar_sim ${LIDAR_SIM_LIBRARIES} )

add_executable(eval_scene_sims scene_sim/eval_scene_sims.cpp)
  target_link_libraries(eval_scene_sims lidar_sim ${LIDAR_SIM_LIBRARIES} )


