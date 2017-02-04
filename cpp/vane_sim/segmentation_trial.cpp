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
#include <lidar_sim/GeometricSegmenter.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // pts from xyz
    // std::string rel_path_xyz = "data/rim_stretch_segment_train.asc";
    std::string rel_path_xyz = "../data/taylorJune2014/sections/world_frame/section_pts_03_world_frame_subsampled.xyz";
    std::vector<std::vector<double> > pts_all = loadPtsFromXYZFile(rel_path_xyz);
    
    int skip = 1;
    std::vector<std::vector<double> > pts = subsampleArray(pts_all, skip);
    
    // segment
    GeometricSegmenter segmenter;
    std::vector<int> segmentation = segmenter.segmentPts(pts);

    std::vector<std::vector<double> > pts_ground = logicalSubsetArray(pts, segmentation);
    std::vector<std::vector<double> > pts_non_ground = logicalSubsetArray(pts, negateLogicalVec(segmentation));

    // viz
    RangeDataVizer vizer;
    vizer.vizSegmentation(pts, segmentation);

    // write results
    std::string rel_path_ground = "data/sections/section_03/section_pts_03_ground.xyz";
    std::string rel_path_non_ground = "data/sections/section_03/section_pts_03_non_ground.xyz";
    writePtsToXYZFile(pts_ground, rel_path_ground);
    writePtsToXYZFile(pts_non_ground, rel_path_non_ground);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
