#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/EllipsoidModeler.h>

using namespace lidar_sim;

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

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

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsNonGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_non_ground.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    int block_id = 8;
    
    // load the ellipsoid models
    std::string rel_path_ellipsoids;
    rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);
    EllipsoidModels ellipsoidModels = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoids);

    // jam models into a modeler
    EllipsoidModeler modeler;
    modeler.setEllipsoidModels(ellipsoidModels);
    modeler.setDebugFlag(1);
   
    // section
    std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // block pts 
    std::string rel_path_block_pts = genRelPathBlock(section_id, block_id);
    std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_block_pts);
    
    // section pts to process
    int num_nbrs = 1;
    std::vector<std::vector<int> > section_nbr_pt_ids; 

    std::tie(section_nbr_pt_ids, std::ignore) = nearestNeighbors(section.m_pts, block_pts, num_nbrs);

    std::vector<std::vector<double> > section_pts_to_process;
    std::vector<int> section_pt_ids_to_process;
    for(size_t i = 0; i < section_nbr_pt_ids.size(); ++i)
	for(size_t j = 0; j < section_nbr_pt_ids[i].size(); ++j)
	{
	    int id = section_nbr_pt_ids[i][j];
	    section_pt_ids_to_process.push_back(id);
	    section_pts_to_process.push_back(
		section.m_pts[id]);
	}

    // write the pts to process
    // std::string rel_path_pts_for_hit_prob = "data/pts_for_hit_prob.xyz";
    // writePtsToXYZFile(section_pts_to_process, rel_path_pts_for_hit_prob);
    
    modeler.calcHitProb(section, section_pt_ids_to_process, imu_pose_server);

    // std::string rel_path_ellipsoids_out = "data/ellipsoids_debug_hit_prob.txt";
    // modeler.writeEllipsoidsToFile(rel_path_ellipsoids_out);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

