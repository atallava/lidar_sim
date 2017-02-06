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
    std::string rel_path_pts = "data/sections/section_03/section_03_block_10_non_ground.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_pts);

    // triangles
    std::string rel_path_ellipsoids = "data/sections/section_03/section_03_block_10_non_ground_ellipsoids.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModelsFromFile(rel_path_ellipsoids);

    // viz
    RangeDataVizer vizer;
    vizer.vizEllipsoidModels(ellipsoid_models, pts);
    return(1);
}
