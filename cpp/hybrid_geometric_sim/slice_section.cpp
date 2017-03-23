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
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

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
    int section_id = 3;
    std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
    SectionLoader section(rel_path_section);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

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
    std::vector<int> ground_blocks_slice {2};
    std::vector<int> ground_blocks_slice_m1 = {1}; // block indexing starts from 1
    int slice_start_node_id = block_node_ids_ground[ground_blocks_slice_m1[0]][0];
    int slice_end_node_id = block_node_ids_ground[ground_blocks_slice_m1.back()][1];
    // get those posns
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

    int n_poses_to_sim = 40*100;
    int section_log_id_skip = slice_n_section_log_ids/n_poses_to_sim;

    // true and sim pts
    std::vector<std::vector<double> > slice_pts;
    for(size_t i = slice_start_section_log_id; i <= (size_t)slice_end_section_log_id; i += section_log_id_skip)
    {
    	double t = section.m_packet_timestamps[i];
    	std::vector<std::vector<double> > this_pts = section.getPtsAtTime(t);
	slice_pts.insert(slice_pts.end(), this_pts.begin(), this_pts.end());
    }

    // write out
    std::string rel_path_pts = "data/section_03_slice_variable.xyz";
    writePtsToXYZFile(slice_pts, rel_path_pts);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
