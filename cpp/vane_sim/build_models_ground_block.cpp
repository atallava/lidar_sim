#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/TriangleModeler.h>

using namespace lidar_sim;

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground.xyz";

    return ss.str();
}

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles_train.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    int block_id = 2;

    std::cout << "processing block " << block_id << "..." << std::endl;

    // model block
    std::string rel_path_pts = genRelPathBlock(section_id, block_id);

    TriangleModeler modeler;
    modeler.setDebugFlag(1);
    modeler.createTriangleModels(rel_path_pts);
	
    // TODO: include prob hit calc

    // write out
    std::string rel_path_triangles = genRelPathTriangles(section_id, block_id);
    modeler.writeTrianglesToFile(rel_path_triangles);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}

