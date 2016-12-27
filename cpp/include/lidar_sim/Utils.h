#pragma once
#include <string>

namespace lidar_sim {
    std::string exec(const char* cmd);	
    int getNumLinesInFile(std::string rel_path_file);
    std::string genPCDHeader(int num_pts);
}
