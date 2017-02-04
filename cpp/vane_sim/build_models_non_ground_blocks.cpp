#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/EllipsoidModeler.h>

using namespace lidar_sim;

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    // TODO: function to determine what block ids are
    std::vector<int> block_ids;
    for(size_t i = 1; i <= 23; ++i)
	block_ids.push_back(i);
    
    // loop over blocks
    for(size_t i = 0; i < block_ids.size(); ++i)
    {
	// model each block
	int block_id = block_ids[i];
	std::cout << "processing block " << block_id << "..." << std::endl;

	std::string rel_path_pts = genRelPathBlock(section_id, block_id);

	EllipsoidModeler modeler;
	modeler.setDebugFlag(1);
	modeler.createEllipsoidModels(rel_path_pts);
	
        // TODO: include prob hit calc

	// write out
	std::string rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);
	modeler.writeEllipsoidsToFile(rel_path_ellipsoids);
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

