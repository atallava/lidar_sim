#include <tuple>
#include <ctime>

#include <vtkProperty.h>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
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

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // pts
    std::string rel_path_pts = "data/sections/section_03/section_03_block_03_ground.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // triangles
    std::string rel_path_triangles = "data/sections/section_03/section_03_block_03_ground_triangles.txt";
    TriangleModelSim sim;
    sim.loadTriangleModels(rel_path_triangles);

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;

    vtkSmartPointer<vtkActor> pts_actor = 
	vizer.m_points_actor_server.genPointsActor(pts);
    pts_actor->GetProperty()->SetColor(1, 1, 1);
    actors.push_back(pts_actor);

    vtkSmartPointer<vtkActor> tri_actor = 
	vizer.m_triangles_actor_server.genTrianglesActor(sim.m_triangles, sim.m_fit_pts);
    tri_actor->GetProperty()->SetColor(0.5451, 0.2706, 0.0745);
    actors.push_back(tri_actor);

    vizer.takeItAway(actors);
    
    return(1);
}
