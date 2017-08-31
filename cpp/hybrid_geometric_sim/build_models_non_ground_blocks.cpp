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
       << "/hg_sim/blocks_info" 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

std::string genRelPathNonGroundBlocksDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim/blocks_info";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, std::string sim_version, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/version_" << sim_version
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
       << "/hg_sim/blocks_info/block_node_ids_non_ground.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::string sim_version = "310817";

    int section_id = 3;
    std::string rel_path_non_ground_blocks_dir = genRelPathNonGroundBlocksDir(section_id);
    std::vector<int> block_ids = getNonGroundBlockIds(rel_path_non_ground_blocks_dir, section_id);
    
    // section
    std::string rel_path_section = genRelPathSection(section_id);
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // blocks info
    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);
    std::vector<std::vector<int> > block_node_ids_non_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    // loop over blocks
    // todo: cleanup
    // for(size_t i = 0; i < block_ids.size(); ++i)
    for(size_t i = 0; i < 1; ++i)
    {
	// model each block
	int block_id = block_ids[i];
	std::cout << "processing block " << block_id << "..." << std::endl;

	std::string rel_path_pts = genRelPathBlock(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	EllipsoidModeler modeler;
	modeler.setVerbosity(1);
	modeler.createEllipsoidModels(rel_path_pts);
	
	// calc hit prob
	// calc section pts to process
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
	modeler.calcHitProb(section, section_pt_ids_to_process, imu_pose_server);

	// write out
	std::string rel_path_ellipsoids = genRelPathEllipsoids(section_id, sim_version, block_id);
	modeler.writeEllipsoidsToFile(rel_path_ellipsoids);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

