#include <lidar_sim/Test.h>

#include <lidar_sim/Visualizer.h>

using namespace lidar_sim;

bool Test::testVizPCD(std::string file_name)
{
    Visualizer viz;
    viz.visualize(file_name);
    return true;
}

