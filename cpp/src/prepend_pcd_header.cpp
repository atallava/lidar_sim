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
    std::string rel_path_xyz_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_xyz_post = "_world_frame_subsampled.xyz";

    std::string rel_path_pcd_pre = "../data/taylorJune2014/sections/world_frame/section_";
    std::string rel_path_pcd_post = "_world_frame_subsampled.pcd";

    std::vector<int> section_ids;
    for (size_t i = 1; i <= 14; ++i) 
	section_ids.push_back(i);
    
    // loop over sections
    clock_t start_time = clock();
    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	// open xyz file
	int section_id = section_ids[i];
	std::ostringstream ss;
	ss << rel_path_xyz_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_xyz_post;
	std::string rel_path_xyz = ss.str();
	std::ifstream xyz_file(rel_path_xyz);
	std::cout << "Reading from: " << rel_path_xyz << std::endl;

	// open pcd file
	ss.str("");
	ss.clear();
	ss << rel_path_pcd_pre << std::setw(2) << std::setfill('0') << section_id << rel_path_pcd_post;
	std::string rel_path_pcd = ss.str();
	std::ofstream pcd_file(rel_path_pcd);
	std::cout << "Writing to: " << rel_path_pcd << std::endl;

	int num_pts = getNumLinesInFile(rel_path_xyz); 
	std::string pcd_header = genPCDHeader(num_pts);
	
	// write header
	pcd_file << pcd_header;

	// rest of file
	std::string current_line;
	while(std::getline(xyz_file,current_line))
	{
	    pcd_file << current_line << std::endl;
	}

	// close files
	xyz_file.close();
	pcd_file.close();
    }
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
