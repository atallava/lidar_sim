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

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genRelPathSectionPts(int section_id)
{
    std::ostringstream ss;
    ss << "data/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

std::string genRelPathNonGroundPts(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/hg_sim/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    int section_id = 3;
    int block_id = 8;

    // pts
    std::string rel_path_pts = genRelPathNonGroundPts(section_id, block_id);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // ellipsoids
    // std::string rel_path_ellipsoids = genRelPathEllipsoids(section_id, block_id);
    std::string rel_path_ellipsoids = "data/hg_optim/ellipsoids.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoids);

    // viz
    RangeDataVizer vizer;
    vizer.vizEllipsoidModels(ellipsoid_models, pts);
    return(1);
}
