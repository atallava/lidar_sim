#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
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
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

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
    
    int section_id = 3;
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    for(size_t i = 1; i <= 23; ++i)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_id, i));

    std::vector<std::string> rel_path_triangle_model_blocks;
    for(size_t i = 1; i <= 4; ++i)
    	rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_id, i));

    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    
    sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

    std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(section_id);
    std::string rel_path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(section_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground, rel_path_block_node_ids_non_ground);

    // viz
    RangeDataVizer vizer;
    vizer.vizSectionModels(sim);

    return(1);
}
