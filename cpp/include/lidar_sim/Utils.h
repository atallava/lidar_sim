#pragma once
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    std::string exec(const char* cmd);	
    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor);
    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output);
}
