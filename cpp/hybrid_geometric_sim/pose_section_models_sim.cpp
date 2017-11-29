#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // sim object
    int section_id = 3;
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    for(size_t i = 1; i <= 23; ++i)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_id, i));

    std::vector<std::string> rel_path_triangle_model_blocks;
    for(size_t i = 1; i <= 4; ++i)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_id, i));

    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(section_id);
    std::string rel_path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(section_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // specify pose
    std::vector<double> imu_pose = {455.729, -498.582, -5.825, 0.0397752, 0.022933, 5.07743};
    std::vector<std::vector<double> > sim_pts_all;
    std::vector<int> hit_flag;
    std::tie(sim_pts_all, hit_flag) = sim.simPtsGivenPose(imu_pose);
    std::vector<std::vector<double> > sim_pts = logicalSubsetArray(sim_pts_all, hit_flag);

    bool use_hit_flag = true;

    // rays
    std::vector<double> ray_origin = {imu_pose[1], imu_pose[0], imu_pose[2]};
    LaserCalibParams laser_calib_params;
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, laser_calib_params);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
