#include <string>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/RangeDataVizer.h>

using namespace lidar_sim;

int main(int argc, char**argv)
{
    std::string rel_path_ply = "data/3d_models/Tree2.ply";
    TriangleModels mesh_model = loadMeshModelFromPly(rel_path_ply);

    RangeDataVizer vizer;
    vizer.vizTriangles(mesh_model.m_triangles, mesh_model.m_fit_pts);
}
