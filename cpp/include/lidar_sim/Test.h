#pragma once
#include <string>

namespace lidar_sim {
    class Test {
    public:
        bool testVizPCD(std::string file_name, int is_rgb);
        bool testVizPoints();
    };
}
