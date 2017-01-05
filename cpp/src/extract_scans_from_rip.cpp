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

    std::ifstream rip_file(rel_path_rip);
    std::cout << "Reading from: " << rel_path_rip << std::endl;

    std::ofstream scans_file(rel_path_scans);
    std::cout << "Writing to: " << rel_path_scans << std::endl;

    // transforms
    Eigen::Matrix<float,4,4> T_imu_velodyne;
    T_imu_velodyne << 
	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984,
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396,
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000,
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
    
    Eigen::Matrix<float,4,4> T_velodyne_imu;
    // T_velodyne_imu = T_imu_velodyne.inverse();
    // TODO: fix this
    T_velodyne_imu = T_imu_velodyne;

    Eigen::Matrix<float,4,4> T_base_world;
    // TODO: fix tbase world
    // T_base_world << 
    // 	1, 0, 0, 0,
    // 	0, -1, 0, 0,
    // 	0, 0, -1, 0,
    // 	0, 0, 0, 1;

    // does nothing if identity
    T_base_world << 
    	1, 0, 0, 0,
    	0, 1, 0, 0,
    	0, 0, 1, 0,
    	0, 0, 0, 1;

    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);
	
    clock_t start_time = clock();
    std::string current_line;
    while(std::getline(rip_file, current_line))
    {
	double data;
	double packet_timestamp_int;
	double packet_timestamp_frac;
	double packet_timestamp;
	int packet_id;
	std::istringstream iss(current_line);
	iss >> packet_id; 
	if (packet_id < start_packet_id)
	    continue;
	if (packet_id > end_packet_id)
	    break;

	iss >> packet_timestamp_int; // integer part
	iss >> packet_timestamp_frac; // fractional timestamp

	// quirk in data
	// TODO: use the packet timestamp file directly
	packet_timestamp = packet_timestamp_int + packet_timestamp_frac*1e-9 - 1;

	Eigen::Matrix<float,4,1> pt_laser;
	iss >> pt_laser[0]; // sensor frame x
	iss >> pt_laser[1]; // sensor frame y
	iss >> pt_laser[2]; // sensor frame z
	pt_laser[3] = 1;

	Eigen::Matrix<float,4,1> pt_imu;
	pt_imu = T_velodyne_imu*pt_laser;
	    
	Eigen::Matrix<float,4,4> T_imu_base = imu_pose_server.getTransfAtTime(packet_timestamp);
	Eigen::Matrix<float,4,1> pt_base;
	pt_base = T_imu_base*pt_imu;
	    
	Eigen::Matrix<float,4,1> pt_world;
	pt_world = T_base_world*pt_base;

	// TODO: shortcutting transformations
	pt_world = pt_base;

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
