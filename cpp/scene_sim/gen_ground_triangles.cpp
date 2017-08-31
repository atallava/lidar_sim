#include <ctime>

#include <lidar_sim/GroundTrianglesGenerator.h>
#include <lidar_sim/DataProcessingUtils.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathTrianglesFitPts(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles_fit_pts.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 4;
    int block_id = 1;

    // get imu posn nodes for sec 4
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genPathImuPosnNodes(section_id), 3);

    // create triangles
    GroundTrianglesGenerator gen;
    gen.setDebugFlag(1);
    gen.createTriangleModels(imu_posn_nodes);

    // block node ids for ground
    // entire ground in one block
    std::vector<std::vector<int> > block_node_ids_ground(1, std::vector<int>(2));
    block_node_ids_ground[0][0] = 0;
    block_node_ids_ground[0][1] = imu_posn_nodes.size();

    // write out
    writePtsToXYZFile(block_node_ids_ground, genPathBlockNodeIdsGround(section_id));
    gen.writeTrianglesToFile(genRelPathTriangles(section_id, block_id));
    gen.writeTrianglesFitPtsToFile(genRelPathTrianglesFitPts(section_id, block_id));
    
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}


