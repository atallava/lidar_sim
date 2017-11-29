#include <string>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/MeshModelSim.h>

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
    sim.setDeterministicSim(true);

    std::vector<double> ray_origin {-2, 0, 1};
    std::vector<double> ray_dirn {1, 0, 0};
    std::vector<int> object_ids = sim.calcObjectIdsForSim(ray_origin, ray_dirn);

    std::vector<std::vector<double> > ray_dirns = wrapDataInVec(ray_dirn);
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = sim.simPtsGivenRays(ray_origin, ray_dirns);
    std::cout << "sim pt: " << std::endl;
    dispVec(sim_pts[0]);
    std::cout << "hit flag: " << hit_flag[0] << std::endl;
    
    return(1);
}
