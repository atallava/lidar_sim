#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <stdexcept>

#include <Eigen/Dense>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserCalibParams.h>

using namespace lidar_sim;

std::string genRelPathSectionLaserFrame(int section_id)
{
    std::ostringstream ss;

    ss << "../data/taylorJune2014/sections/laser_frame/section_"
       << std::setw(2) << std::setfill('0') << section_id 
       << ".xyz";

    return ss.str();
}

std::string genRelPathSectionWorldFrame(int section_id)
{
    std::ostringstream ss;

    ss << "../data/taylorJune2014/sections/world_frame/section_"
       << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame.xyz";

    return ss.str();
}

std::string genRelPathSectionPtsWorldFrame(int section_id)
{
    std::ostringstream ss;

    ss << "../data/taylorJune2014/sections/world_frame/section_pts_"
       << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame.xyz";

    return ss.str();
}

int main() {
    std::string rel_path_section_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_section_post = "_subsampled.xyz";
    
    std::string rel_path_section_world_frame_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_section_world_frame_post = "_world_frame_subsampled.xyz";

    std::string rel_path_section_pts_world_frame_pre = "../data/taylorJune2014/sections/world_frame/section_pts_";
    std::string rel_path_section_pts_world_frame_post = "_world_frame_subsampled.xyz";
    
    std::vector<int> section_ids;
    for (size_t i = 1; i <= 14; ++i) 
	section_ids.push_back(i);
    
    // transforms
    // cetin's notation: imu_to_velodyne, my notation: velodyne_imu. same transform!
    Eigen::Matrix<float,4,4> T_velodyne_imu;
    T_velodyne_imu << 
	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984,
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396,
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000,
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000;

    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    LaserCalibParams laser_calib_params;

    // ranges less than this distance discarded
    double min_range_filter = 2;
	
    // loop over sections
    clock_t start_time = clock();
    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	int section_id = i;

	// open section file
	std::string rel_path_section = genRelPathSectionLaserFrame(section_id);
	std::ifstream section_file(rel_path_section);
	std::cout << "Reading from: " << rel_path_section << std::endl;

	// open section world frame file
	std::string rel_path_section_world_frame = genRelPathSectionWorldFrame(section_id);
	std::ofstream section_world_frame_file(rel_path_section_world_frame);
	std::cout << "Writing to: " << rel_path_section_world_frame << std::endl;

	// open section detail world frame file
	std::string rel_path_section_pts_world_frame = genRelPathSectionPtsWorldFrame(section_id);
	std::ofstream section_pts_world_frame_file(rel_path_section_pts_world_frame);
	std::cout << "Writing to: " << rel_path_section_pts_world_frame << std::endl;

	// loop over section points
	std::string current_line;
	while(std::getline(section_file, current_line))
	{
	    double packet_timestamp_sec;
	    double packet_timestamp_nanosec;
	    double packet_timestamp;
	    int packet_id;
	    std::istringstream iss(current_line);

	    // packet id
	    iss >> packet_id; 

	    // packet timestamp
	    iss >> packet_timestamp_sec;
	    iss >> packet_timestamp_nanosec;
	    packet_timestamp = packet_timestamp_sec + packet_timestamp_nanosec*1e-9;

	    // pt in laser frame
	    Eigen::Matrix<float,4,1> pt_laser;
	    iss >> pt_laser[0];
	    iss >> pt_laser[1];
	    iss >> pt_laser[2];
	    pt_laser[3] = 1;

	    // pt in imu frame
	    Eigen::Matrix<float,4,1> pt_imu;
	    pt_imu = T_velodyne_imu*pt_laser;
	    
	    // pt in world frame
	    std::vector<double> imu_pose = imu_pose_server.getPoseAtTime(packet_timestamp);
	    Eigen::Matrix<float,4,4> T_imu_world = imu_pose_server.getTransfAtTime(packet_timestamp);
	    Eigen::Matrix<float,4,1> pt_world;
	    pt_world = T_imu_world*pt_imu;

	    // measured range
	    std::vector<double> laser_posn = laserPosnFromImuPose(imu_pose, laser_calib_params);
	    std::vector<double> pt_world_stl(3,0);
	    for(size_t i = 0; i < 3; ++i)
		pt_world_stl[i] = pt_world(i);
	    double measured_range = euclideanDist(laser_posn, pt_world_stl);
	    if (measured_range < min_range_filter)
		continue;
	    
	    // write to file
	    std::ostringstream ss;
	    std::string output_line;
	    ss.str("");
	    ss.clear();
	    ss << packet_id << " ";
	    ss << std::setprecision(20) << packet_timestamp_sec << " " << std::setprecision(20) << packet_timestamp_nanosec << " ";
	    ss << pt_world[0] << " " << pt_world[1] << " " << pt_world[2] << std::endl;
	    output_line = ss.str();
	    section_world_frame_file << output_line;

	    // write to pts-only file
	    ss.str("");
	    ss.clear();
	    ss << pt_world[0] << " " << pt_world[1] << " " << pt_world[2] << std::endl;
	    section_pts_world_frame_file << ss.str();
	    
	    // debug
	    // std::cout << "laser point: " << pt_laser << std::endl;
	    // std::cout << "T laser imu: " << T_velodyne_imu << std::endl;
	    // std::cout << "imu point: " << pt_imu << std::endl;
	    // std::cout << "T imu: " << T_imu << std::endl;
	    // std::cout << "world point: " << pt_world << std::endl;
	    // break;
	}

	// close files
	section_file.close();
	section_world_frame_file.close();
	section_pts_world_frame_file.close();
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
