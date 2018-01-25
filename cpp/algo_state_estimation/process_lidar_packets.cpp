#include <iostream>
#include <ctime>

#include <lidar_sim/PoseServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/StateEstPacketAggregator.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;
    std::string path_section = genPathSection(section_id);

    StateEstPacketAggregator aggregator(path_section);
    // aggregator.aggregatePacketsIntoScans();

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
