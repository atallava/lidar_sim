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

#include <lidar_sim/Utils.h>

using namespace lidar_sim;

int main() {
    std::string rel_path_file_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_file_post = ".xyz";

    std::string rel_path_file_subsampled_pre = "../data/taylorJune2014/sections/laser_frame/section_";
    std::string rel_path_file_subsampled_post = "_subsampled.xyz";

    std::vector<int> section_ids;
    for (size_t i = 1; i <= 14; ++i) 
	section_ids.push_back(i);
    
    int max_pts = 1000000;

    // loop over sections
    clock_t start_time = clock();
    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	//input file path
	int section_id = section_ids[i];
	std::ostringstream ss;
	ss << rel_path_file_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_post;
	std::string rel_path_file = ss.str();

	// calculate subsample factor
	int num_pts = getNumLinesInFile(rel_path_file);
	int subsample_factor = floor(num_pts/max_pts);
	// std::cout << subsample_factor << std::endl;

	// output file path
	ss.str("");
	ss.clear();
	ss << rel_path_file_subsampled_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_subsampled_post;
	std::string rel_path_file_subsampled = ss.str();

	// subsample
	subsampleFile(rel_path_file, rel_path_file_subsampled, subsample_factor);
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
