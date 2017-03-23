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

using namespace lidar_sim;

std::string genRelPathTriangles(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground_triangles.txt";

    return ss.str();
}

std::string genRelPathGroundPts(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    int section_id = 3;
    int block_id = 2;

    // pts
    std::string rel_path_pts = genRelPathGroundPts(section_id, block_id);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // triangles
    std::string rel_path_triangles = genRelPathTriangles(section_id, block_id);
    TriangleModelSim sim;
    sim.loadTriangleModels(rel_path_triangles);

    // viz
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;

    vtkSmartPointer<vtkActor> pts_actor = 
    	vizer.m_points_actor_server.genPointsActor(pts);
    pts_actor->GetProperty()->SetColor(1, 1, 1);
    actors.push_back(pts_actor);

    std::vector<vtkSmartPointer<vtkActor> > tri_actors = 
	vizer.m_triangles_actor_server.genTrianglesActors(sim.m_triangles, sim.m_fit_pts, sim.m_hit_prob_vec);
    actors.insert(actors.end(), tri_actors.begin(), tri_actors.end());

    vizer.takeItAway(actors);
    
    return(1);
}
