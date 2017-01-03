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

#include <lidar_sim/Utils.h>

using namespace lidar_sim;

int main() {
    std::string rel_path_section_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_section_post = "_subsampled.xyz";
    
    std::string rel_path_section_world_frame_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_section_world_frame_post = "_world_frame_subsampled.xyz";

    std::string rel_path_section_imu_transfs_pre = "../data/taylorJune2014/sections/imu_transfs/imu_transfs_";
    std::string rel_path_section_imu_transfs_post = ".txt";

    std::vector<int> section_ids;
    for (size_t i = 1; i <= 1; ++i) 
	section_ids.push_back(i);
    
    // transforms
    Eigen::Matrix<float,4,4> T_imu_velodyne;
    T_imu_velodyne << 0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984,
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396,
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000,
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
    Eigen::Matrix<float,4,4> T_velodyne_imu;
    T_velodyne_imu = T_imu_velodyne.inverse();

    // loop over sections
    clock_t start_time = clock();
    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	// open section file
	int section_id = section_ids[i];
	std::ostringstream ss;
	ss << rel_path_section_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_post;
	std::string rel_path_section = ss.str();
	std::ifstream section_file(rel_path_section);
	std::cout << "Reading from: " << rel_path_section << std::endl;

	// open section world frame file
	ss.str("");
	ss.clear();
	ss << rel_path_section_world_frame_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_world_frame_post;
	std::string rel_path_section_world_frame = ss.str();
	std::ofstream section_world_frame_file(rel_path_section_world_frame);
	std::cout << "Writing to: " << rel_path_section_world_frame << std::endl;

	// open imu transfs file
	ss.str("");
	ss.clear();
	ss << rel_path_section_imu_transfs_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_imu_transfs_post;
	std::string rel_path_section_imu_transfs = ss.str();
	std::ifstream section_imu_transfs_file(rel_path_section_imu_transfs);
	std::cout << "Imu transfs file: " << rel_path_section_imu_transfs << std::endl;

	std::string current_line;
	std::string imu_transfs_line;
	int old_packet_id = -1; // just something invalid
	int transf_packet;
	while(std::getline(section_file, current_line))
	{
	    double data;
	    int packet_id;
	    std::istringstream iss(current_line);
	    iss >> packet_id; // packet id
	    iss >> data; // timestamp
	    iss >> data; // timestamp

	    Eigen::Matrix<float,4,1> pt_laser;
	    iss >> pt_laser[0]; // sensor frame x
	    iss >> pt_laser[1]; // sensor frame y
	    iss >> pt_laser[2]; // sensor frame z
	    pt_laser[3] = 1;

	    Eigen::Matrix<float,4,1> pt_imu;
	    pt_imu = T_velodyne_imu*pt_laser;

	    // if new packet id, read next transf
	    Eigen::Matrix<float,4,4> T_imu;
	    if (packet_id != old_packet_id)
	    {
		std::getline(section_imu_transfs_file, imu_transfs_line);
		transf_packet = parseTransfsFileLine(imu_transfs_line, T_imu);
		old_packet_id = packet_id;
	    }

	    // if (transf_packet != packet_id) 
	    // {
	    // 	std::cout << "transf packet: " << transf_packet << " does not match packet id: " << 
	    // 	    packet_id << std::endl;
	    // 	return -1;
	    // }

	    Eigen::Matrix<float,4,1> pt_world;
	    pt_world = T_imu*pt_imu;

	    // write to file
	    std::string output_line;
	    ss.str("");
	    ss.clear();
	    ss << pt_world[0] << " " << pt_world[1] << " " << pt_world[2] << std::endl;
	    output_line = ss.str();
	    section_world_frame_file << output_line;
	    
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
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
