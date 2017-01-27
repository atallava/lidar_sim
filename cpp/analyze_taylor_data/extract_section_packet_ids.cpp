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

#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

int main() {
    std::string rel_path_section_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_section_post = "_subsampled.xyz";
    
    std::string rel_path_section_ids_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_section_ids_post = "_subsampled_packet_ids.txt";

    std::vector<int> section_ids;
    for (size_t i = 1; i <= 1; ++i) 
	section_ids.push_back(i);
    
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

	// open section packet ids file
	ss.str("");
	ss.clear();
	ss << rel_path_section_ids_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_section_ids_post;
	std::string rel_path_section_ids = ss.str();
	std::ofstream section_ids_file(rel_path_section_ids);
	std::cout << "Writing to: " << rel_path_section_ids << std::endl;

	std::string current_line;
	int old_packet_id = -1; // just something invalid
	while(std::getline(section_file, current_line))
	{
	    int packet_id;
	    double packet_read_time_int_part;
	    double packet_read_time_frac_part;
	    std::istringstream iss(current_line);
	    iss >> packet_id; // packet id
	    iss >> packet_read_time_int_part; 
	    iss >> packet_read_time_frac_part;

	    // if new packet id, write to section ids file
	    if (packet_id != old_packet_id)
	    {
		std::string output_line;
		ss.str("");
		ss.clear();
		ss << packet_id << " " ;
		ss << std::setprecision(10) << packet_read_time_int_part << "." << packet_read_time_frac_part << std::endl;
		output_line = ss.str();
		section_ids_file << output_line;

		old_packet_id = packet_id;
	    }
	}

	// close files
	section_file.close();
	section_ids_file.close();
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
