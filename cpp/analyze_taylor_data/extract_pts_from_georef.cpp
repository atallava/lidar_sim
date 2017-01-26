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
    std::string rel_path_input = "../data/taylorJune2014/Velodyne/georef/TaylorGeoreferenced_subsampled.xyz0.xyz";

    int start_count = 1000000;
    int end_count = start_count+1000000-1;

    std::ostringstream ss;
    ss << "../data/taylorJune2014/Velodyne/georef/pts_" << std::setw(8) << std::setfill('0') << start_count << "_"
       << std::setw(8) << std::setfill('0') << end_count << ".xyz";
    std::string rel_path_output = ss.str();
	
    // open georef file
    std::ifstream file_input(rel_path_input);
    std::cout << "Reading from: " << rel_path_input << std::endl;

    // open output file
    std::ofstream file_output(rel_path_output);
    std::cout << "Writing to: " << rel_path_output << std::endl;

    clock_t start_time = clock();

    std::string current_line;
    int count = 0;
    while(std::getline(file_input, current_line))
    {
	count++;
	if (count >= start_count)
	    file_output << current_line << std::endl;
	if (count >= end_count)
	    break;
    }
    file_input.close();
    file_output.close();

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return 0;
}
