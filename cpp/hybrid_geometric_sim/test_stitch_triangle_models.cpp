#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathModelsDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    std::string rel_path_models_dir = genRelPathModelsDir(section_id);
    // find triangle models for this section
    std::vector<std::string> rel_path_triangle_model_blocks;
    std::vector<int> triangle_model_block_ids = 
	getTriangleModelBlockIds(rel_path_models_dir, section_id);
    for(auto i : triangle_model_block_ids)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_id, i));

    std::vector<TriangleModels> triangle_models_vec;
    for(size_t i = 0; i < rel_path_triangle_model_blocks.size(); ++i)
    {
	TriangleModels triangle_models = loadTriangleModelsFromFile(
	    rel_path_triangle_model_blocks[i]);
	triangle_models_vec.push_back(triangle_models);
    }
    TriangleModels triangle_models_all = stitchTriangleModels(triangle_models_vec);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
