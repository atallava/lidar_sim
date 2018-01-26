#include <iostream>
#include <ctime>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PacketsToScanAggregator.h>

using namespace lidar_sim;

std::string genRelPathPackets(int section_id, std::string version, std::string data_source)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version << "/" << data_source
       << "/packets_to_process.txt";

    return ss.str();
}

std::string genRelPathProcessInfo(int section_id, std::string version)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version 
       << "/process_info.txt";

    return ss.str();
}

std::string genRelPathScansWorldFrameDir(int section_id, std::string version, std::string data_source)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version << "/" << data_source
       << "/scans_world_frame";

    return ss.str();
}

std::string genRelPathScansLaserFrameDir(int section_id, std::string version, std::string data_source)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version << "/" << data_source
       << "/scans_laser_frame";

    return ss.str();
}

std::string genRelPathScanPoses(int section_id, std::string version, std::string data_source)
{
    std::ostringstream ss;
    ss << "data/algo_state_estimation/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/version_" << version << "/" << data_source
       << "/scan_poses.txt";

    return ss.str();
}

void mkdirsForScans(int section_id, std::string version, std::string data_source)
{
    // scans world frame dir
    std::string rel_path_dir = 
	genRelPathScansWorldFrameDir(section_id, version, data_source);
    myMkdir(rel_path_dir);

    // scans laser frame dir
    rel_path_dir = 
	genRelPathScansLaserFrameDir(section_id, version, data_source);
    myMkdir(rel_path_dir);
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;
    std::string process_version = "260118";
    std::string data_source = "real";

    std::string rel_path_packets = genRelPathPackets(section_id, process_version, data_source);
    std::string rel_path_process_info = genRelPathProcessInfo(section_id, process_version);
    
    // aggregate packets into scans
    PacketsToScanAggregator aggregator;
    aggregator.setVerbosity(1);
    aggregator.loadPackets(rel_path_packets);
    aggregator.loadProcessInfo(rel_path_process_info);
    aggregator.aggregate();

    // save data
    mkdirsForScans(section_id, process_version, data_source);
    std::string rel_path_scans_world_frame_dir = 
	genRelPathScansWorldFrameDir(section_id, process_version, data_source);
    aggregator.saveScansWorldFrame(rel_path_scans_world_frame_dir);

    std::string rel_path_scans_laser_frame_dir = 
	genRelPathScansLaserFrameDir(section_id, process_version, data_source);
    aggregator.saveScansLaserFrame(rel_path_scans_laser_frame_dir);

    std::string rel_path_scan_poses = 
	genRelPathScanPoses(section_id, process_version, data_source);
    aggregator.saveScanPoses(rel_path_scan_poses);

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
