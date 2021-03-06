#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/TriangleModeler.h>

using namespace lidar_sim;

std::string genRelPathGroundBlocksDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim/blocks_info";

    return ss.str();
}

std::string genRelPathTriangles(int section_id, std::string sim_version, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/version_" << sim_version
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 1;
    std::string rel_path_ground_blocks_dir = genRelPathGroundBlocksDir(section_id);
    std::vector<int> block_ids = getGroundBlockIds(rel_path_ground_blocks_dir, section_id);
    
    // section
    std::string rel_path_section = genPathSection(section_id);
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = genPathPosesLog();
    PoseServer imu_pose_server(rel_path_poses_log);

    // blocks info
    std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(rel_path_imu_posn_nodes, 3);
    std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(section_id);
    std::vector<std::vector<int> > block_node_ids_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));

    // loop over blocks
    std::string hg_sim_version = "130917";
    for(size_t i = 0; i < block_ids.size(); ++i)
    {
	std::cout << "processing block " << i << "..." << std::endl;

	// model each block
	int block_id = block_ids[i];
	std::string rel_path_pts = genPathGroundBlockPts(section_id, block_id);
	std::vector<std::vector<double> > block_pts = loadPtsFromXYZFile(rel_path_pts);

	TriangleModeler modeler;
	modeler.setDebugFlag(1);
	modeler.createTriangleModels(rel_path_pts);
	
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
	std::string rel_path_triangles = genRelPathTriangles(section_id, hg_sim_version, block_id);
	modeler.writeTrianglesToFile(rel_path_triangles);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

