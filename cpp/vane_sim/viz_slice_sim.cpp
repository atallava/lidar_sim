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

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_ground.txt";

    return ss.str();
}

std::string genRelPathBlockNodeIdsNonGround(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/block_node_ids_non_ground.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    // observed pts
    std::string rel_path_pts = "data/section_03_slice_variable.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // sim pts
    std::string rel_path_pts_sim = "data/section_03_slice_sim.xyz";
    std::vector<std::vector<double> > sim_pts = loadPtsFromXYZFile(rel_path_pts_sim);

    // models
    // sim vecs
    int section_id = 3;
    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_ground = genRelPathBlockNodeIdsGround(section_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);

    std::vector<std::vector<int> > block_node_ids_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_ground, 2));
    std::vector<std::vector<int> > block_node_ids_non_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    std::vector<int> ground_blocks {2};
    std::vector<int> ground_blocks_m1 = {1}; // block indexing starts from 1
    std::vector<int> non_ground_blocks_m1 = getOverlappingIntervals(block_node_ids_non_ground, 
								    logicalSubsetArray(block_node_ids_ground, 
										       genLogicalVecFromIds(ground_blocks_m1, block_node_ids_ground.size())));
    std::vector<int> non_ground_blocks = non_ground_blocks_m1; 
    std::for_each(non_ground_blocks.begin(), non_ground_blocks.end(), [](int &n){ n++; });

    std::vector<TriangleModelSim> tri_sims;
    for(auto i : ground_blocks)
    {
	TriangleModelSim sim;
	sim.loadTriangleModels(genRelPathTriangles(section_id, i));
	tri_sims.push_back(sim);
    }

    std::vector<EllipsoidModelSim> ellipsoid_sims;
    for(auto i : non_ground_blocks)
    {
	EllipsoidModelSim sim;
	sim.loadEllipsoidModels(genRelPathEllipsoids(section_id, i));
	ellipsoid_sims.push_back(sim);
    }

    // vizer
    RangeDataVizer vizer;
    std::vector<vtkSmartPointer<vtkActor> > actors;

    // tri actors
    std::vector<vtkSmartPointer<vtkActor> > tri_actors
	= vizer.genTriangleModelsActors(tri_sims);
    actors.insert(actors.end(), tri_actors.begin(), tri_actors.end());

    // ellipsoid actors
    // note that not all ellipsoids may be pushed into actors
    std::vector<vtkSmartPointer<vtkActor> > ellipsoid_actors
	= vizer.genEllipsoidModelsActors(ellipsoid_sims);
    actors.insert(actors.end(), ellipsoid_actors.begin(), ellipsoid_actors.end());

    // measured pts
    vtkSmartPointer<vtkActor> pts_actor
	= vizer.genPointsActor(pts);
    pts_actor->GetProperty()->SetColor(1, 1, 1);
    actors.push_back(pts_actor);

    // sim pts
    vtkSmartPointer<vtkActor> sim_pts_actor
	= vizer.genPointsActor(sim_pts);
    sim_pts_actor->GetProperty()->SetColor(1, 0, 0);
    actors.push_back(sim_pts_actor);

    // fire up
    vizer.takeItAway(actors);

    return(1);
}

