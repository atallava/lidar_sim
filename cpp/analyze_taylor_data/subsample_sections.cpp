#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/SectionLoader.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame.xyz";

    return ss.str();
}

std::string genRelPathSectionSubsampled(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}


int main() {
    std::vector<int> section_ids;
    // todo: process all sections
    for (size_t i = 1; i <= 1; ++i) 
	section_ids.push_back(i);
    
    int max_pts = 1000000;

    clock_t start_time = clock();
    // can i even load a full section?
    std::string rel_path_section = genRelPathSection(3);
    SectionLoader section(rel_path_section);

    // todo: uncomment
    // // loop over sections
    // for (size_t i = 0; i < section_ids.size(); ++i)
    // {
    // 	//input file path
    // 	int section_id = section_ids[i];
    // 	std::ostringstream ss;
    // 	ss << rel_path_file_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_post;
    // 	std::string rel_path_file = ss.str();

    // 	// calculate subsample factor
    // 	int num_pts = getNumLinesInFile(rel_path_file);
    // 	int subsample_factor = floor(num_pts/max_pts);
    // 	// std::cout << subsample_factor << std::endl;

    // 	// output file path
    // 	ss.str("");
    // 	ss.clear();
    // 	ss << rel_path_file_subsampled_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_subsampled_post;
    // 	std::string rel_path_file_subsampled = ss.str();

    // 	// subsample
    // 	subsampleFile(rel_path_file, rel_path_file_subsampled, subsample_factor);
    // }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
