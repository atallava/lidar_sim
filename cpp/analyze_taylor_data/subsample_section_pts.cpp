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

using namespace lidar_sim;

std::string genRelPathSectionPts(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_pts_"
       << std::setw(2) << std::setfill('0') << section_id << "_world_frame.xyz";

    return ss.str();
}

std::string genRelPathSubsampledPts(int section_id)
{
    std::ostringstream ss;
    ss << "../data/taylorJune2014/sections/world_frame/section_pts_"
       << std::setw(2) << std::setfill('0') << section_id << "_subsampled.xyz";

    return ss.str();
}

int main() {
    clock_t start_time = clock();

    std::vector<int> section_ids;
    for (size_t i = 1; i <= 14; ++i) 
	section_ids.push_back(i);

    int subsample_factor = 30;

    for (size_t i = 0; i < section_ids.size(); ++i)
    {
	int section_id = section_ids[i];

	std::string rel_path_file = genRelPathSectionPts(section_id);
	std::string rel_path_file_subsampled = genRelPathSubsampledPts(section_id);

	subsampleFile(rel_path_file, rel_path_file_subsampled, subsample_factor);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
