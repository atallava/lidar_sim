#include <tuple>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/RangeDataVizer.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/TriangleModelSim.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/PoseServer.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pick a ground block
    int section_id = 3;
    int block_id = 4;

    // get the triangle models for that block
    std::string rel_path_triangle_models = genRelPathTriangles(section_id, block_id);
    TriangleModelSim sim;
    sim.loadTriangleModels(rel_path_triangle_models);

    // calc the avg fit pts z value in block
    double avg_z_fit_pts = 0;
    for(size_t i = 0; i < sim.m_fit_pts.size(); ++i)
	avg_z_fit_pts += sim.m_fit_pts[i][2];

    avg_z_fit_pts /= sim.m_fit_pts.size();

    std::cout << "avg z fit pts: " << avg_z_fit_pts << std::endl;

    // imu nodes, block ids
    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);
    std::vector<std::vector<int> > block_node_ids_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));

    // calc avg z values of nodes in that block
    // sorry, blocks index begins at 1
    double avg_z_imu_posns = 0;
    for(size_t i = block_node_ids_ground[block_id-1][0]; 
	i <= (size_t)block_node_ids_ground[block_id-1][1]; ++i)
	avg_z_imu_posns += imu_posn_nodes[i][2];

    avg_z_imu_posns /= (imu_posn_nodes[block_id-1][1]-imu_posn_nodes[block_id-1][0]+1);

    std::cout << "avg z imu posns: " << avg_z_imu_posns << std::endl;
    std::cout << "avg ground depth from imu: " << (avg_z_imu_posns - avg_z_fit_pts) << std::endl;

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}
