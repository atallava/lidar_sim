#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/MathUtils.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/EllipsoidModeler.h>

using namespace lidar_sim;

std::string genRelPathPrimitivePts(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/pts/" << class_name << "_" << element_id << ".asc";

    return ss.str();
}

std::string genRelPathBlock(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

    return ss.str();
}

std::string genRelPathEllipsoids(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids_train.txt";

    return ss.str();
}

std::string genRelPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

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
    clock_t start_time = clock();

    // class names
    std::vector<std::string> primitive_classes {"low_shrub", "low_shrub_patch", "medium_shrub", "medium_shrub_patch", "thin_shrub", "large_shrub", "large_shrub_patch",
	    "medium_tree", "large_tree"};

    int section_id = 3;

    // element ids
    // todo: code for this
    std::vector<std::vector<int> > element_ids;
    {
	std::vector<int> low_shrub_ids;
	for(size_t i = 1; i <= 11; ++i)
	    low_shrub_ids.push_back(i);
	element_ids.push_back(low_shrub_ids);

	std::vector<int> low_shrub_patch_ids;
	for(size_t i = 1; i <= 12; ++i)
	    low_shrub_patch_ids.push_back(i);
	element_ids.push_back(low_shrub_patch_ids);

	std::vector<int> medium_shrub_ids;
	for(size_t i = 1; i <= 21; ++i)
	{
	    if (i == 9)
		continue;
	    medium_shrub_ids.push_back(i);
	}
	element_ids.push_back(medium_shrub_ids);

	std::vector<int> medium_shrub_patch_ids;
	for(size_t i = 1; i <= 11; ++i)
	    medium_shrub_patch_ids.push_back(i);
	element_ids.push_back(medium_shrub_patch_ids);

	std::vector<int> thin_shrub_ids;
	for(size_t i = 1; i <= 2; ++i)
	    thin_shrub_ids.push_back(i);
	element_ids.push_back(thin_shrub_ids);

	std::vector<int> large_shrub_ids;
	for(size_t i = 1; i <= 4; ++i)
	    large_shrub_ids.push_back(i);
	element_ids.push_back(large_shrub_ids);

	std::vector<int> large_shrub_patch_ids;
	for(size_t i = 1; i <= 4; ++i)
	    large_shrub_patch_ids.push_back(i);
	element_ids.push_back(large_shrub_patch_ids);

	std::vector<int> medium_tree_ids;
	for(size_t i = 1; i <= 22; ++i)
	    medium_tree_ids.push_back(i);
	element_ids.push_back(medium_tree_ids);

	std::vector<int> large_tree_ids;
	for(size_t i = 1; i <= 11; ++i)
	    large_tree_ids.push_back(i);
	element_ids.push_back(large_tree_ids);
    } 

    // gen path to the pts in each

    // get all the ellipsoid models
    std::string rel_path_imu_posn_nodes = genRelPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_non_ground = genRelPathBlockNodeIdsNonGround(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);
    std::vector<std::vector<int> > block_node_ids_non_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    // for each set of pts
    std::vector<std::string> rel_paths_primitive_pts;
    for(size_t i = 0; i < primitive_classes.size(); ++i)
	for(size_t j = 0; j < element_ids[i].size(); ++j)
	{
	    std::vector<std::vector<double> > pts =
		loadPtsFromXYZFile(genRelPathPrimitivePts(section_id, primitive_classes[i], element_ids[i][j]));
	    
	    // calc mean
	    std::vector<double> mu = calcPtsMean(pts);
	    std::vector<std::vector<double> > principal_axes = calcPrincipalAxes2D(pts);
	    
	    exit(0);
	}


    // first, get an oriented bounding box

    // decide which ellipsoid model blocks to search in

    // search in those, which mean falls in the bounding box

    // figure out a pose

    // write out those ellipsoid models

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}



