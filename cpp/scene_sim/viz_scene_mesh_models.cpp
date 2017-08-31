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
#include <lidar_sim/MeshModelSim.h>

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathObjectMeshesDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim";

    return ss.str();
}

std::string genRelPathObjectMesh(int section_id, int object_id)
{
   std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/mesh_model_sim/" << object_id << ".txt";

    return ss.str();
}

std::string genPathHgModelsDir(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
       << "/hg_sim";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // // load section
    int section_sim_id = 4;
    // std::string rel_path_section = genPathSection(section_sim_id);
    // SectionLoader section(rel_path_section);

    // sim object
    int section_hg_models_id = 4;
    std::string rel_path_hg_models_dir = genPathHgModelsDir(section_hg_models_id);

    // find object meshes
    std::vector<std::string> rel_path_object_meshes;
    std::string rel_path_object_meshes_dir = genRelPathObjectMeshesDir(section_sim_id);
    std::vector<int> object_mesh_ids = 
	getObjectMeshIds(rel_path_object_meshes_dir);
    for(auto i : object_mesh_ids)
	rel_path_object_meshes.push_back(genRelPathObjectMesh(section_sim_id, i));

    // find triangle models for this section
    std::vector<std::string> rel_path_ground_triangle_model_blocks;
    std::vector<int> ground_triangle_model_block_ids = 
	getTriangleModelBlockIds(rel_path_hg_models_dir, section_hg_models_id);
    for(auto i : ground_triangle_model_block_ids)
    	rel_path_ground_triangle_model_blocks.push_back(genRelPathTriangles(section_hg_models_id, i));

    // create sim object
    MeshModelSim sim;
    sim.loadTriangleModelBlocks(rel_path_ground_triangle_model_blocks); 
    sim.loadObjectMeshes(rel_path_object_meshes);

    sim.setDeterministicSim(false);

    std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(section_hg_models_id);
    std::string rel_path_block_node_ids_ground = genPathBlockNodeIdsGround(section_hg_models_id);

    sim.loadBlockInfo(rel_path_imu_posn_nodes, rel_path_block_node_ids_ground);

    // pose server
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);

    RangeDataVizer vizer;
    vizer.vizMeshModelSim(sim);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
