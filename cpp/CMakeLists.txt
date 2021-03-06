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
find_package( Boost COMPONENTS filesystem system thread log regex program_options REQUIRED ) 
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND LIDAR_SIM_LIBRARIES ${Boost_LIBRARIES} )
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_thread" ) # Force linking against thread
# list( APPEND LIDAR_SIM_LIBRARIES "-lboost_system" ) # Force linking against system

# CGAL
find_package( CGAL)
include( ${CGAL_USE_FILE} ) 
include( CGAL_CreateSingleSourceCGALProgram ) 
# message( STATUS "CGAL FOUND: " ${CGAL_FOUND} )
# message( FATAL_ERROR )

# EIGEN
set( EIGEN_INCLUDE_DIR "/usr/include/eigen3" )
include_directories( ${EIGEN_INCLUDE_DIR} )

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

# NLOPT
set( NLOPT_LIBRARY "/usr0/local/lib/libnlopt.a" )
list( APPEND LIDAR_SIM_LIBRARIES ${NLOPT_LIBRARY} )

# OPENMP
find_package(OpenMP)
if (OPENMP_FOUND)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# lidar_sim_classification
set( LIDAR_SIM_CLASSIFICATION_INCLUDE_DIR "/usr0/home/atallav1/lidar_sim_classification/cpp/include" )
include_directories( ${LIDAR_SIM_CLASSIFICATION_INCLUDE_DIR} )
set( LIDAR_SIM_CLASSIFICATION_LIBRARY "/usr0/home/atallav1/lidar_sim_classification/cpp/lib/liblidar_sim_classification.so" )
set( LIDAR_SIM_CLASSIFICATION_LIBRARY_DIR "/usr0/home/atallav1/lidar_sim_classification/cpp/lib" )
link_directories( ${LIDAR_SIM_CLASSIFICATION_LIBRARY_DIR} )
list( APPEND LIDAR_SIM_LIBRARIES ${LIDAR_SIM_CLASSIFICATION_LIBRARY} )

# c++ 11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 
# todo: only for valgrind. delete else.
# the -DCGAL... flag is because without it, valgrind encounters a cgal assertion error. some bug.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O0 -frounding-math -DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON") 

add_definitions( -Wall -std=c++0x )

# This project's headers
set( LIDAR_SIM_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )
include_directories(${LIDAR_SIM_INCLUDE_DIRECTORIES})

# Add files here to compile them
add_library(lidar_sim SHARED 	

  src/PoseServer.cpp
  src/FrameTimeServer.cpp
  src/DataProcessingUtils.cpp
  src/VizUtils.cpp

  src/MathUtils.cpp
  src/LaserCalibParams.cpp
  src/LaserUtils.cpp
  src/PoseUtils.cpp
  src/ClusteringUtils.cpp
  src/ModelingUtils.cpp
  src/RayDirnServer.cpp
  src/FlannDatasetWrapper.cpp

  src/SectionLoader.cpp
  src/GeometricSegmenter.cpp
  src/TriangleModeler.cpp
  src/EllipsoidModeler.cpp
  src/EllipsoidModelSim.cpp
  src/TriangleModelSim.cpp
  src/SectionModelSim.cpp
  src/SimDetail.cpp
  src/EllipsoidSimNbrServer.cpp
  src/TriangleSimNbrServer.cpp

  src/NearestNeighborSim.cpp

  src/PtsError.cpp
  src/OptimProgress.cpp
  src/OptimAssistant.cpp

  src/GroundTrianglesGenerator.cpp
  src/OrientedBox.cpp
  src/SceneObjectServer.cpp
  src/MeshModelSim.cpp

  # algo state estimation
  src/PacketsToScanAggregator.cpp
  src/AlgoStateEstUtils.cpp
  )
 
####################################################################################################

# Executables

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

# add_executable(subsample_sections analyze_taylor_data/subsample_sections.cpp)
#   target_link_libraries(subsample_sections lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(subsample_section_pts analyze_taylor_data/subsample_section_pts.cpp)
#   target_link_libraries(subsample_section_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(transform_sections_to_world analyze_taylor_data/transform_sections_to_world.cpp)
#   target_link_libraries(transform_sections_to_world lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(analyze_scanning_pattern analyze_taylor_data/analyze_scanning_pattern.cpp)
#   target_link_libraries(analyze_scanning_pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

# flann examples

## pattern

# # add_executable(pattern flann_examples/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(test_mm_flann_leak flann_examples/test_mm_flann_leak.cpp)
#   target_link_libraries(test_mm_flann_leak lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(test_flann_matrix flann_examples/test_flann_matrix.cpp)
#   target_link_libraries(test_flann_matrix lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(test_flann_dataset_wrapper flann_examples/test_flann_dataset_wrapper.cpp)
#   target_link_libraries(test_flann_dataset_wrapper lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

# nlopt examples

## pattern

# # add_executable(pattern nlopt_examples/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(trial nlopt_examples/trial.cpp)
#   target_link_libraries(trial lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

# omp examples

## pattern

# # add_executable(pattern omp_examples/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(sin_table omp_examples/sin_table.cpp)
#   target_link_libraries(sin_table lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(omp_hello_world omp_examples/omp_hello_world.cpp)
#   target_link_libraries(omp_hello_world lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(phrases_test omp_examples/phrases_test.cpp)
#   target_link_libraries(phrases_test lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################
 
# hybrid_geometric sim
 
## pattern

# # add_executable(pattern hybrid_geometric_sim/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

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

# add_executable(build_models_non_ground_block hybrid_geometric_sim/build_models_non_ground_block.cpp)
#   target_link_libraries(build_models_non_ground_block lidar_sim ${LIDAR_SIM_LIBRARIES} )

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

# add_executable(section_slice_real_pts hybrid_geometric_sim/section_slice_real_pts.cpp)
#   target_link_libraries(section_slice_real_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(slice_pts_in_blocks hybrid_geometric_sim/slice_pts_in_blocks.cpp)
#   target_link_libraries(slice_pts_in_blocks lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_blocks_real_pts hybrid_geometric_sim/section_blocks_real_pts.cpp)
#   target_link_libraries(section_blocks_real_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

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

# add_executable(slice_hg_sim hybrid_geometric_sim/slice_hg_sim.cpp)
#   target_link_libraries(slice_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(blocks_hg_sim hybrid_geometric_sim/blocks_hg_sim.cpp)
#   target_link_libraries(blocks_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(test_stitch_triangle_models hybrid_geometric_sim/test_stitch_triangle_models.cpp)
#   target_link_libraries(test_stitch_triangle_models lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(test_triangle_sim_nbr_server hybrid_geometric_sim/test_triangle_sim_nbr_server.cpp)
#   target_link_libraries(test_triangle_sim_nbr_server lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_hg_sim hybrid_geometric_sim/section_hg_sim.cpp)
#   target_link_libraries(section_hg_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## nbr sim

# add_executable(poses_nbr_sim hybrid_geometric_sim/poses_nbr_sim.cpp)
#   target_link_libraries(poses_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(slice_nbr_sim hybrid_geometric_sim/slice_nbr_sim.cpp)
#   target_link_libraries(slice_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(section_nbr_sim hybrid_geometric_sim/section_nbr_sim.cpp)
#   target_link_libraries(section_nbr_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## eval sim

# add_executable(eval_sims hybrid_geometric_sim/eval_sims.cpp)
#   target_link_libraries(eval_sims lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(sim_optim hybrid_geometric_sim/sim_optim.cpp)
#   target_link_libraries(sim_optim lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

## scene sim

# hg sim

# add_executable(slice_hg_scene_sim scene_sim/slice_hg_scene_sim.cpp)
#   target_link_libraries(slice_hg_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )
  
# mesh model sim

# add_executable(test_mesh_model_sim scene_sim/test_mesh_model_sim.cpp)
#   target_link_libraries(test_mesh_model_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(slice_mm_scene_sim scene_sim/slice_mm_scene_sim.cpp)
#   target_link_libraries(slice_mm_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

# nbr sim

# add_executable(slice_nbr_scene_sim scene_sim/slice_nbr_scene_sim.cpp)
#   target_link_libraries(slice_nbr_scene_sim lidar_sim ${LIDAR_SIM_LIBRARIES} )

## eval

add_executable(eval_scene_sims scene_sim/eval_scene_sims.cpp)
target_link_libraries(eval_scene_sims lidar_sim ${LIDAR_SIM_LIBRARIES} )

####################################################################################################

## algo state estimation

## pattern

# # add_executable(pattern algo_state_estimation/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(aggregate_packets_to_scans algo_state_estimation/aggregate_packets_to_scans.cpp)
# target_link_libraries(aggregate_packets_to_scans lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(create_real_packets_to_process algo_state_estimation/create_real_packets_to_process.cpp)
# target_link_libraries(create_real_packets_to_process lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(hg_sim_debug algo_state_estimation/hg_sim_debug.cpp)
#   target_link_libraries(hg_sim_debug lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(mm_sim_debug algo_state_estimation/mm_sim_debug.cpp)
#   target_link_libraries(mm_sim_debug lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(create_hg_sim_packets_to_process algo_state_estimation/create_hg_sim_packets_to_process.cpp)
# target_link_libraries(create_hg_sim_packets_to_process lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(create_hg_sim_packets_to_process_omp algo_state_estimation/create_hg_sim_packets_to_process_omp.cpp)
# target_link_libraries(create_hg_sim_packets_to_process_omp lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(create_mm_sim_packets_to_process algo_state_estimation/create_mm_sim_packets_to_process.cpp)
# target_link_libraries(create_mm_sim_packets_to_process lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(create_mm_sim_packets_to_process_omp algo_state_estimation/create_mm_sim_packets_to_process_omp.cpp)
# target_link_libraries(create_mm_sim_packets_to_process_omp lidar_sim ${LIDAR_SIM_LIBRARIES})

# add_executable(write_imu_poses_for_section algo_state_estimation/write_imu_poses_for_section.cpp)
# target_link_libraries(write_imu_poses_for_section lidar_sim ${LIDAR_SIM_LIBRARIES})

####################################################################################################

## algo classification

## pattern

# # add_executable(pattern algo_classification/pattern.cpp)
# #   target_link_libraries(pattern lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(get_non_ground_from_ground_pts algo_classification/get_non_ground_from_ground_pts.cpp)
#   target_link_libraries(get_non_ground_from_ground_pts lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(gen_runs_vehicle_posn_nodes algo_classification/gen_runs_vehicle_posn_nodes.cpp)
#   target_link_libraries(gen_runs_vehicle_posn_nodes lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_blocks_ground_gascola algo_classification/build_blocks_ground_gascola.cpp)
#   target_link_libraries(build_blocks_ground_gascola lidar_sim ${LIDAR_SIM_LIBRARIES} )

# add_executable(build_models_ground_blocks_gascola algo_classification/build_models_ground_blocks_gascola.cpp)
#   target_link_libraries(build_models_ground_blocks_gascola lidar_sim ${LIDAR_SIM_LIBRARIES} )

add_executable(hg_sim_for_classification algo_classification/hg_sim_for_classification.cpp)
  target_link_libraries(hg_sim_for_classification lidar_sim ${LIDAR_SIM_LIBRARIES} )

add_executable(mm_sim_for_classification algo_classification/mm_sim_for_classification.cpp)
  target_link_libraries(mm_sim_for_classification lidar_sim ${LIDAR_SIM_LIBRARIES} )

