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

std::string genBlockPtsRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << ".xyz";

    return ss.str();
}

std::string genBlockNonGroundRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id
       << "_non_ground.xyz";

    return ss.str();
}

std::string genBlockGroundRelPath(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_pts_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id
       << "_ground.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    int section_id = 3;
    int block_id = 13;
    std::string rel_path_block_pts = genBlockPtsRelPath(section_id, block_id);
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_block_pts);

    RangeDataVizer vizer;

    // viz pts
    // vizer.vizPts(pts);

    std::cout << "segmenting..." << std::endl;
    // segment
    GeometricSegmenter segmenter;
    segmenter.setDebugFlag(1);
    std::vector<int> segmentation = segmenter.segmentPts(pts);

    // viz segmentation
    vizer.vizSegmentation(pts, segmentation);

    // write pts to file
    // std::vector<std::vector<double> > pts_ground = logicalSubsetArray(pts, segmentation);
    // std::vector<std::vector<double> > pts_non_ground = logicalSubsetArray(pts, negateLogicalVec(segmentation));
    // writePtsToXYZFile(pts_ground, genBlockGroundRelPath(section_id, block_id));
    // writePtsToXYZFile(pts_non_ground, genBlockNonGroundRelPath(section_id, block_id));
    
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
