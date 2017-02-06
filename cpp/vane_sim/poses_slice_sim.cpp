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

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_ground.txt";

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

    // load section
    std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    int section_id = 3;

    // imu posn nodes
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);

    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);

    std::vector<std::vector<int> > block_node_ids_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));
    std::vector<std::vector<int> > block_node_ids_non_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    // specify node ids
    int ground_block_slice = 2;
    int slice_start_node_id = block_node_ids_ground[ground_block_slice][0];
    int slice_end_node_id = block_node_ids_ground[ground_block_slice][1];
    // get those poses
    std::vector<double> slice_start_imu_posn = imu_posn_nodes[slice_start_node_id];
    std::vector<double> slice_end_imu_posn = imu_posn_nodes[slice_end_node_id];

    // get slice times
    // section imu poses
    std::vector<std::vector<double> > section_imu_posns;
    for(size_t i = 0; i < section.m_packet_timestamps.size(); ++i)
    {
	double t = section.m_packet_timestamps[i];
	std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(t);
	section_imu_posns.push_back(posnFromImuPose(imu_pose));
    }

    // nearest posns in section
    std::vector<std::vector<int> > nn_ids;
    std::vector<std::vector<double> > wrapped_slice_posns;
    wrapped_slice_posns.push_back(slice_start_imu_posn);
    wrapped_slice_posns.push_back(slice_end_imu_posn);
    std::tie(nn_ids, std::ignore) = nearestNeighbors(section_imu_posns, wrapped_slice_posns, 1);

    // slice times
    int slice_start_section_log_id = nn_ids[0][0];
    int slice_end_section_log_id = nn_ids[1][0];
    int slice_n_section_log_ids = slice_end_section_log_id - slice_start_section_log_id;
    double slice_start_time = section.m_packet_timestamps[slice_start_section_log_id];
    double slice_end_time = section.m_packet_timestamps[slice_end_section_log_id];
    double slice_duration = slice_end_time - slice_start_time;

    int n_poses_to_sim = 100;
    int section_log_id_skip = slice_n_section_log_ids/n_poses_to_sim;

    // sim object
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    // sim from ellipsoids close to ground block in slice
    std::vector<int> non_ground_blocks_for_sim;
    std::vector<int> non_ground_blocks_for_sim_flag(block_node_ids_non_ground.size(), 0);
    for(size_t i = 0; i < block_node_ids_non_ground.size(); i++)
    {
	bool condn_1 = ((slice_start_node_id <= block_node_ids_non_ground[i][0]) &&
			(block_node_ids_non_ground[i][0] <= slice_end_node_id));
	bool condn_2 = ((slice_start_node_id <= block_node_ids_non_ground[i][1]) &&
			(block_node_ids_non_ground[i][1] <= slice_end_node_id));

	if (condn_1 || condn_2)
	{
	    non_ground_blocks_for_sim.push_back(i);
	    non_ground_blocks_for_sim_flag[i] = 1;
	}
    }

    for(auto i : non_ground_blocks_for_sim)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_id, i));

    std::vector<std::string> rel_path_triangle_model_blocks;
    int ground_block_for_sim = ground_block_slice;
    rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_id, ground_block_slice));

    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);
    // sim.m_imu_posn_nodes = imu_posn_nodes;
    // sim.m_block_node_ids_ground = block_node_ids_ground;
    // sim.m_block_node_ids_non_ground = logicalSubsetArray(block_node_ids_non_ground,
    // 							non_ground_blocks_for_sim_flag);

    // debug
    // std::cout << "ground block slice: " << ground_block_slice << std::endl;
    // std::cout << "ground block for sim: " << ground_block_for_sim << std::endl;
    // std::cout << "non ground blocks for sim: " << std::endl;
    // dispVec(non_ground_blocks_for_sim);
    
    // true and sim pts
    // todo: scanning patterns are different
    // section pts are subsampled
    // and there is skip
    std::vector<std::vector<double> > slice_pts;
    std::vector<std::vector<double> > slice_pts_sim;

    for(size_t i = slice_start_section_log_id; i <= slice_end_section_log_id; i += section_log_id_skip)
    {
    	double t = section.m_packet_timestamps[i];
    	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);
	slice_pts.insert(slice_pts.end(), this_pts.begin(), this_pts.end());

	std::vector<double> this_pose = imu_pose_server.getPoseAtTime(t);
	std::vector<std::vector<double> > this_pts_sim_all;
	std::vector<int> hit_flag;
	std::tie(this_pts_sim_all, hit_flag) = sim.simPtsGivenPose(this_pose);
	std::vector<std::vector<double> > this_pts_sim = logicalSubsetArray(this_pts_sim_all, hit_flag);
	slice_pts_sim.insert(slice_pts_sim.end(), this_pts_sim.begin(), this_pts_sim.end());
    }

    // write out
    std::string rel_path_pts = "data/section_03_slice.xyz";
    writePtsToXYZFile(slice_pts, rel_path_pts);

    std::string rel_path_pts_sim = "data/section_03_slice_sim.xyz";
    writePtsToXYZFile(slice_pts_sim, rel_path_pts_sim);

    // which blocks are activated?

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
