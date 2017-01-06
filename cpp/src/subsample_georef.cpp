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
    std::string rel_path_file = "../data/taylorJune2014/Velodyne/TaylorGeoreferenced.xyz0.xyz";
    std::string rel_path_file_subsampled = "../data/taylorJune2014/Velodyne/TaylorGeoreferenced_subsampled.xyz0.xyz";

    clock_t start_time = clock();
    int subsample_factor = 30;
    subsampleFile(rel_path_file, rel_path_file_subsampled, subsample_factor);
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
