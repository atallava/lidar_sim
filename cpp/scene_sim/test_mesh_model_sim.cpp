#include <string>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/MeshModelSim.h>
#include <lidar_sim/RangeDataVizer.h>

using namespace lidar_sim;

int main(int argc, char**argv)
{
    std::vector<std::string> rel_path_object_meshes;
    std::string rel_path_object_meshes_dir = "data/sections/section_04/scene_object_meshes";
    rel_path_object_meshes.push_back(rel_path_object_meshes_dir + "/tree_1.txt");
    rel_path_object_meshes.push_back(rel_path_object_meshes_dir + "/tree_2.txt");
    rel_path_object_meshes.push_back(rel_path_object_meshes_dir + "/shrub_1.txt");

    MeshModelSim sim;
    sim.loadObjectMeshes(rel_path_object_meshes);

    // RangeDataVizer vizer;
    // vizer.vizObjectMeshes(sim.m_object_mesh_sims);

    std::vector<double> ray_origin {2, -5, 0};
    std::vector<double> ray_dirn {0, 1, 0};
    std::vector<int> object_ids = sim.calcObjectIdsForSim(ray_origin, ray_dirn);
}
