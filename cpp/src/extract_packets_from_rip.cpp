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
#include <lidar_sim/Utils.h>

using namespace lidar_sim;

int main() {
    std::string rel_path_rip = "../data/taylorJune2014/Velodyne/Velodyne_Rip.xyz";

    int start_packet_id = 190000;
    int end_packet_id = 192000;

    std::ostringstream ss;
    ss << "../data/taylorJune2014/Velodyne/packets/packets_" << std::setw(7) << std::setfill('0') << start_packet_id << "_"
       << std::setw(7) << std::setfill('0') << end_packet_id << ".xyz";
    std::string rel_path_scans = ss.str();

    // rip file
    std::ifstream rip_file(rel_path_rip);
    std::cout << "Reading from: " << rel_path_rip << std::endl;

    // output file
    std::ofstream scans_file(rel_path_scans);
    std::cout << "Writing to: " << rel_path_scans << std::endl;

    // transforms
    // cetin's notation: imu_to_velodyne, my notation: velodyne_imu. same transform!
    Eigen::Matrix<float,4,4> T_velodyne_imu;
    T_velodyne_imu << 
	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984,
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396,
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000,
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
    
    // imu poses
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);
	
    clock_t start_time = clock();
    std::string current_line;
    while(std::getline(rip_file, current_line))
    {
	double packet_timestamp_sec;
	double packet_timestamp_nanosec;
	double packet_timestamp;
	int packet_id;
	std::istringstream iss(current_line);

	// packet id
	iss >> packet_id; 
	if (packet_id < start_packet_id)
	    continue;
	if (packet_id > end_packet_id)
	    break;

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
	Eigen::Matrix<float,4,4> T_imu_world = imu_pose_server.getTransfAtTime(packet_timestamp);
	Eigen::Matrix<float,4,1> pt_world;
	pt_world = T_imu_world*pt_imu;

	// write to file
	std::string output_line;
	ss.str("");
	ss.clear();
	ss << pt_world[0] << " " << pt_world[1] << " " << pt_world[2] << std::endl;
	output_line = ss.str();
	scans_file << output_line;
	    
	// debug
	// std::cout << "laser point: " << pt_laser << std::endl;
	// std::cout << "T laser imu: " << T_velodyne_imu << std::endl;
	// std::cout << "imu point: " << pt_imu << std::endl;
	// std::cout << "T imu: " << T_imu << std::endl;
	// std::cout << "world point: " << pt_world << std::endl;
	// break;
    }
    
    // close files
    rip_file.close();
    scans_file.close();

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
