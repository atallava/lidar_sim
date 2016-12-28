#pragma once
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    std::string exec(const char* cmd);	
    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
    int parseTransfsFileLine(std::string line, Eigen::Matrix<float,4,4>& T_imu);
}
