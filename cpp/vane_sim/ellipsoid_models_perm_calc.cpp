#include <string>
#include <iostream>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>

using namespace lidar_sim;

int main(int argc, char **argv)
{
    // load ellipsoids
    std::string rel_path_ellipsoid_models = "data/ellipsoid_models.txt";
    EllipsoidModels ellipsoid_models = 
	loadEllipsoidModels(rel_path_ellipsoid_models);
    
    // load section
    std::string rel_path_section = "data/section_03_world_frame_subsampled_timed.xyz";
    SectionLoader section(rel_path_section);

    // pts from xyz
    std::string rel_path_xyz = "data/rim_stretch_veg_train.asc";
    std::vector<std::vector<double> > train_pts = loadPtsFromXYZFile(rel_path_xyz);

    // section ids to process
    std::vector<int> section_pt_ids_to_process = nearestNeighbors(section.m_pts, train_pts);

    // std::vector<std::vector<double> > small_pts;
    // small_pts.push_back(section.m_pts[74]);
    // small_pts.push_back(section.m_pts[8671]);
    // std::vector<int> section_pt_ids_to_process = nearestNeighbors(train_pts, small_pts);


    // std::cout << "small pts: " << std::endl;
    // dispMat(small_pts);
    std::cout << "nearest nbrs: " << std::endl;
    dispVec(section_pt_ids_to_process);
}

