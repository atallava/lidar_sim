#include <iostream>
#include <ctime>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PacketsToScanAggregator.h>
#include <lidar_sim/AlgoStateEstUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;
    std::string scans_version = "300118";
    std::string data_source = "real";
    std::string source_version = "";

    std::string rel_path_packets = 
	algo_state_est::genRelPathPacketsToProcess(section_id, scans_version, data_source, source_version);
    std::string rel_path_process_info = 
	algo_state_est::genRelPathProcessInfo(section_id, scans_version);
    
    // aggregate packets into scans
    PacketsToScanAggregator aggregator;
    aggregator.setVerbosity(1);
    aggregator.loadPackets(rel_path_packets);
    aggregator.loadProcessInfo(rel_path_process_info);
    aggregator.aggregate();

    // save data
    algo_state_est::mkdirsForScans(section_id, scans_version, data_source, source_version);
    std::string rel_path_scans_world_frame_dir = 
	algo_state_est::genRelPathScansWorldFrameDir(section_id, scans_version, data_source, source_version);
    aggregator.saveScansWorldFrame(rel_path_scans_world_frame_dir);

    std::string rel_path_scans_laser_frame_dir = 
	algo_state_est::genRelPathScansLaserFrameDir(section_id, scans_version, data_source, source_version);
    aggregator.saveScansLaserFrame(rel_path_scans_laser_frame_dir);

    std::string rel_path_scan_poses = 
	algo_state_est::genRelPathScanPoses(section_id, scans_version, data_source, source_version);
    aggregator.saveScanPoses(rel_path_scan_poses);

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
