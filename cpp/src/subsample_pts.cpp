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
	// open input file
	int section_id = section_ids[i];
	std::ostringstream ss;
	ss << rel_path_file_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_post;
	std::string rel_path_file = ss.str();
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;

	// calculate subsample factor
	int num_pts = getNumLinesInFile(rel_path_file);
	int subsample_factor = floor(num_pts/max_pts);
	// std::cout << subsample_factor << std::endl;

	// open section world frame file
	ss.str("");
	ss.clear();
	ss << rel_path_file_subsampled_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_file_subsampled_post;
	std::string rel_path_file_subsampled = ss.str();
	std::ofstream subsampled_file(rel_path_file_subsampled);
	std::cout << "Writing to: " << rel_path_file_subsampled << std::endl;
	std::cout << "Subsampling factor: " << subsample_factor << std::endl;

	std::string current_line;
	int count = 0;
	while(std::getline(file,current_line))
	{
	    if (count % subsample_factor == 0)
		subsampled_file << current_line << std::endl;

	    count++;
	}

	// close files
	file.close();
	subsampled_file.close();
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
