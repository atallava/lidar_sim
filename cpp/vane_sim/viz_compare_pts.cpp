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
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // load pts1
    std::string rel_path_pts1 = "data/rim_stretch_veg_validation.asc";
    std::vector<std::vector<double> > pts1 = loadPtsFromXYZFile(rel_path_pts1);

    // load pts2
    std::string rel_path_pts2 = "data/rim_stretch_veg_validation_sim.xyz";
    // std::string rel_path_pts2 = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > pts2 = loadPtsFromXYZFile(rel_path_pts2);

    // viz
    RangeDataVizer vizer;
    vizer.vizComparePts(pts1, pts2);
    
    return(1);
}
