#include <lidar_sim/Test.h>

#include <lidar_sim/Visualizer.h>

using namespace lidar_sim;

bool Test::testVizPCD(std::string file_name, int is_rgb = 1)
{
    std::cout << "Visualizing: " << file_name << std::endl;
    Visualizer viz;
    viz.visualize(file_name, is_rgb);
    return true;
}

