#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
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

std::string genBlockRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;

    // load section
    std::string rel_path_section = genSectionRelPath(section_id);
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    // imu posn nodes
    std::vector<std::vector<double> > imu_posn_nodes;
    double dt = 1; // in s

    double prev_t = section.m_packet_timestamps[0]-dt;
    for(size_t i = 0; i < section.m_packet_timestamps.size(); ++i)
    {
    	double t = section.m_packet_timestamps[i];
    	if ( (t-prev_t) >= dt )
    	{
    	    std::vector<double> imu_posn = posnFromImuPose(imu_pose_server.getPoseAtTime(t));
    	    imu_posn_nodes.push_back(imu_posn);
    	    prev_t = t;
    	}
    }

    // viz these nodes
    RangeDataVizer vizer;
    // vizer.vizPts(imu_posn_nodes);

    // pts
    std::string rel_path_pts = "data/sections/section_03/section_pts_03_non_ground.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // nearest neighbor for pts in nodes
    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > nn_dists;
    std::tie(nn_ids, nn_dists) = nearestNeighbors(imu_posn_nodes, pts, 1);
    
    // for each node, which pts are in it
    std::vector<std::vector<int> > node_pts_map;
    // TODO: vector of empty vectors. better way?
    for(size_t i = 0; i < imu_posn_nodes.size(); ++i)
	node_pts_map.push_back(std::vector<int> ());
    
    for(size_t i = 0; i < pts.size(); ++i)
	node_pts_map[nn_ids[i][0]].push_back(i);
    
    // block size
    int pts_per_block = 2e4;

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

    std::cout << "block node ids: " << std::endl;
    dispMat(block_node_ids);

    std::cout << "imu_posn_nodes: " << std::endl;
    dispMat(imu_posn_nodes);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
