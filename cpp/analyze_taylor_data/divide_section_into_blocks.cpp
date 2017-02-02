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
#include <lidar_sim/FrameTimeServer.h>

using namespace lidar_sim;

std::string genSectionRelPath(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_" 
       << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genBlockRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << ".xyz";

    return ss.str();
}

std::string genBlockPtsRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << ".xyz";

    return ss.str();
}

int main() {
    clock_t start_time = clock();

    int section_id = 3;

    int max_lines_in_block = 5e4; // needs to be tuned
    
    std::string rel_path_section = genSectionRelPath(section_id);
    std::ifstream section_file(rel_path_section);

    int block_id = 1;

    std::string rel_path_block = genBlockRelPath(section_id, block_id);
    std::ofstream block_file(rel_path_block);
    std::cout << "Writing to: " << rel_path_block << std::endl;
    
    std::string rel_path_block_pts = genBlockPtsRelPath(section_id, block_id);
    std::ofstream block_pts_file(rel_path_block_pts);
    std::cout << "Writing to: " << rel_path_block_pts << std::endl;

    std::string current_line;
    int count = 0;
    while(std::getline(section_file, current_line))
    {
	block_file << current_line << std::endl;
	block_pts_file << getPtsLineFromSectionLine(current_line) << std::endl;
	count++;

	if (count == max_lines_in_block)
	{
	    // open new file
	    block_file.close();
	    block_pts_file.close();
	    block_id++;

	    rel_path_block = genBlockRelPath(section_id, block_id);
	    block_file.open(rel_path_block);
	    std::cout << "Writing to: " << rel_path_block << std::endl;

	    rel_path_block_pts = genBlockPtsRelPath(section_id, block_id);
	    block_pts_file.open(rel_path_block_pts);
	    std::cout << "Writing to: " << rel_path_block_pts << std::endl;

	    // reset count
	    count = 0;
	}
    }
    block_file.close();

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
