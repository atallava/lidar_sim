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
#include <lidar_sim/EllipsoidModeler.h>
#include <lidar_sim/TriangleModeler.h>

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
    // std::string rel_path_block_pts = genBlockPtsRelPath(section_id, block_id);
    std::string rel_path_block_pts = "data/block_trial.xyz";
    std::vector<std::vector<double> > pts = loadPtsFromXYZFile(rel_path_block_pts);

    RangeDataVizer vizer;

    // viz pts
    // vizer.vizPts(pts);

    // segment pts
    std::cout << "segmenting..." << std::endl;
    GeometricSegmenter segmenter;
    segmenter.setDebugFlag(1);
    std::vector<int> segmentation = segmenter.segmentPts(pts);

    // viz segmentation
    vizer.vizSegmentation(pts, segmentation);

    // write segmented pts to file
    std::vector<std::vector<double> > pts_ground = logicalSubsetArray(pts, segmentation);
    std::vector<std::vector<double> > pts_non_ground = logicalSubsetArray(pts, negateLogicalVec(segmentation));
    std::string rel_path_ground_pts = "data/block_trial_ground.xyz";
    std::string rel_path_non_ground_pts = "data/block_trial_non_ground.xyz";
    writePtsToXYZFile(pts_ground, rel_path_ground_pts);
    writePtsToXYZFile(pts_non_ground, rel_path_non_ground_pts);

    // build ellipsoid models
    EllipsoidModeler ellipsoid_modeler;
    ellipsoid_modeler.setDebugFlag(1);
    ellipsoid_modeler.createEllipsoidModels(rel_path_non_ground_pts);
    
    // hit prob
    std::string rel_path_section = "data/section_03_world_frame_subsampled_timed.xyz";
    std::string rel_path_poses_log = "../data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
    PoseServer imu_pose_server(rel_path_poses_log);
    ellipsoid_modeler.calcHitProb(rel_path_section, imu_pose_server);
    
    // viz ellipsoids
    vizer.vizEllipsoidModels(ellipsoid_modeler.m_ellipsoid_models, ellipsoid_modeler.m_pts);

    // save ellipsoids
    std::string rel_path_ellipsoid_models = "data/block_trial_ellipsoid_models.txt";
    ellipsoid_modeler.writeEllipsoidModelsToFile(rel_path_ellipsoid_models);

    // build tri models
    TriangleModeler triangle_modeler;
    triangle_modeler.setDebugFlag(1);
    triangle_modeler.createTriangleModels(rel_path_ground_pts);

    // hit prob + range
    // assuming that ellipsoids and triangles will use the same section
    triangle_modeler.calcHitProb(rel_path_section, imu_pose_server);

    // viz triangles
    vizer.vizTriangles(triangle_modeler.m_triangles, triangle_modeler.m_pts);

    // save triangles
    std::string rel_path_triangle_models = "data/block_trial_triangle_models.txt";
    triangle_modeler.writeTrianglesToFile(rel_path_triangle_models);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
    
    return(1);
}
