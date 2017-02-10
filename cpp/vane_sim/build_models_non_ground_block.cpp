#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
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
    int block_id = 1;
    
    std::cout << "processing block " << block_id << "..." << std::endl;

    std::string rel_path_pts = genRelPathBlock(section_id, block_id);

    EllipsoidModeler modeler;
    modeler.setDebugFlag(1);
    modeler.createEllipsoidModels(rel_path_pts);

    // // prob hit calc
    // // section
    // std::string rel_path_section = "data/section_03_world_frame_subsampled.xyz";
    // SectionLoader section(rel_path_section);

    // // pose server
    // std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    // PoseServer imu_pose_server(rel_path_poses_log);

    // // blocks info
    // std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    // std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);
    // std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);
    // std::vector<std::vector<int> > block_node_ids_non_ground = 
    // 	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    // // slice imu posns
    // std::vector<std::vector<double> > slice_imu_posns;
    // // blocks are indexed from 1. sorry.
    // for(size_t i = 0; i < 2; ++i)
    // 	slice_imu_posns.push_back(imu_posn_nodes[block_node_ids_non_ground[block_id-1][i]]);

    // // slice section log ids
    // int slice_start_section_log_id;
    // int slice_end_section_log_id;

    // std::tie(slice_start_section_log_id, slice_end_section_log_id) =
    // 	section.getLogIdsBracketingImuPosns(slice_imu_posns, imu_pose_server);

    // // subsample log ids
    // int max_section_packets_to_process = 1e4;
    // int skip = (slice_end_section_log_id - slice_start_section_log_id)/max_section_packets_to_process;
    // if (skip < 1)
    // 	skip = 1;
    // std::vector<int> section_pt_ids_to_process;
    // for(size_t i = slice_start_section_log_id; i <= (size_t)slice_end_section_log_id; i += skip)
    // {
    // 	double t = section.m_packet_timestamps[i];
    // 	std::vector<int> pt_ids = section.getPtIdsAtTime(t);
    // 	section_pt_ids_to_process.insert(section_pt_ids_to_process.end(), 
    // 					 pt_ids.begin(), pt_ids.end());
    // }

    // modeler.calcHitProb(section, section_pt_ids_to_process, imu_pose_server);

    // write out
    std::string rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);
    modeler.writeEllipsoidsToFile(rel_path_ellipsoids);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

