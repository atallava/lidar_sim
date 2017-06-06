#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/GeometricSegmenter.h>

using namespace lidar_sim;

std::string genSectionRelPath(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathSectionPtsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/ground_segmentation/section_pts_" << std::setw(2) << std::setfill('0') << section_id
       << "_ground.asc";

    return ss.str();
}

std::string genBlockRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground.xyz";

    return ss.str();
}

std::string genRelPathBlockNodeIdsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_ground.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;

    // load section
    std::string rel_path_section = genSectionRelPath(section_id);
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // imu posn nodes
    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);

    // section ground pts
    std::string rel_path_pts = genRelPathSectionPtsGround(section_id);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // nearest neighbor for pts in nodes
    std::vector<std::vector<int> > nn_ids;
    std::tie(nn_ids, std::ignore) = nearestNeighbors(imu_posn_nodes, pts, 1);
    
    // for each node, which pts are in it
    std::vector<std::vector<int> > node_pts_map(imu_posn_nodes.size(), 
						std::vector<int> ());
    
    for(size_t i = 0; i < pts.size(); ++i)
	node_pts_map[nn_ids[i][0]].push_back(i);
    
    // block size
    int pts_per_block = 1e5;

    // write blocks
    std::vector<std::vector<int> > block_node_ids;
    std::vector<int> current_block_node_ids(2,0);
    current_block_node_ids[0] = 0;

    int block_id = 1;

    std::string rel_path_block = genBlockRelPath(section_id, block_id);
    std::ofstream block_file(rel_path_block);
    std::cout << "Writing to: " << rel_path_block << std::endl;
    
    std::string current_line;
    int pts_in_current_block = 0;
    for(size_t i = 0; i < node_pts_map.size(); ++i)
    {
	for(size_t j = 0; j < node_pts_map[i].size(); ++j)
	{
	    std::vector<double> pt = pts[node_pts_map[i][j]];
	    block_file << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
	}
	pts_in_current_block += node_pts_map[i].size();

	if (pts_in_current_block >= pts_per_block)
	{
	    if (i == node_pts_map.size()-1)
		break;

	    // block node ids
	    current_block_node_ids[1] = i;
	    block_node_ids.push_back(current_block_node_ids);
	    current_block_node_ids[0] = i+1;

	    // open new file
	    block_file.close();
	    block_id++;

	    rel_path_block = genBlockRelPath(section_id, block_id);
	    block_file.open(rel_path_block);
	    std::cout << "Writing to: " << rel_path_block << std::endl;

	    // reset counter
	    pts_in_current_block = 0;
	}
    }
    current_block_node_ids[1] = node_pts_map.size()-1;
    block_node_ids.push_back(current_block_node_ids);
    block_file.close();

    // write block node ids;
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_id);
    writePtsToXYZFile(block_node_ids, rel_path_block_node_ids_ground);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
