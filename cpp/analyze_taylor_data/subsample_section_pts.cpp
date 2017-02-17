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
       << std::setw(2) << std::setfill('0') << section_id << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathSubsampledPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_pts_"
       << std::setw(2) << std::setfill('0') << section_id << "_world_frame_subsampled_1e5.xyz";

    return ss.str();
}

int main() {
    clock_t start_time = clock();

    int section_id = 8;
    int max_pts = 1e5;

    //input file path
    std::string rel_path_file = genRelPathSectionPts(section_id);

    // calculate subsample factor
    int num_pts = getNumLinesInFile(rel_path_file);
    int subsample_factor = floor(num_pts/max_pts);

    std::string rel_path_file_subsampled = genRelPathSubsampledPts(section_id);

    // subsample
    subsampleFile(rel_path_file, rel_path_file_subsampled, subsample_factor);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
