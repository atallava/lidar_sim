#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/ModelingUtils.h>
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
#include <lidar_sim/OrientedBox.h>
#include <lidar_sim/SectionModelSim.h>

using namespace lidar_sim;

std::string genRelPathPrimitivePts(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/pts/" << class_name << "_" << element_id << ".asc";

    return ss.str();
}

std::string genRelPathPrimitiveEllipsoids(int section_id, std::string class_name, int element_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/primitives/ellipsoids/" << class_name << "_" << element_id << ".txt";

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
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

std::string genPathImuPosnNodes(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/imu_posn_nodes.txt";

    return ss.str();
}

std::string genPathBlockNodeIdsNonGround(int section_id)
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
    // todo: automate this
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

    // load ellipsoids models
    // block info 
    std::string rel_path_imu_posn_nodes = genPathImuPosnNodes(section_id);
    std::string rel_path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(section_id);
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(rel_path_imu_posn_nodes, 3);
    std::vector<std::vector<int> > block_node_ids_non_ground = 
	doubleToIntArray(loadArray(rel_path_block_node_ids_non_ground, 2));

    // paths
    std::vector<std::string> rel_path_ellipsoid_model_blocks;
    std::vector<EllipsoidModels> ellipsoid_models;
    for(size_t i = 1; i <= 23; ++i)
    	rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_id, i));

    // sim
    SectionModelSim sim;
    sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
    
    // loop over primitive pts
    std::vector<std::string> rel_paths_primitive_pts;
    for(size_t i = 0; i < primitive_classes.size(); ++i)
	for(size_t j = 0; j < element_ids[i].size(); ++j)
	{
	    std::vector<std::vector<double> > primitive_pts =
		loadPtsFromXYZFile(genRelPathPrimitivePts(section_id, primitive_classes[i], element_ids[i][j]));
	    
	    // oriented bounding box
	    OrientedBox obb = calcObb(primitive_pts);

	    EllipsoidModels primitive_ellipsoid_models;
	    
	    // todo: search in relevant ellipsoids blocks only
	    for(size_t k = 0; k < sim.m_ellipsoid_model_sims.size(); ++k)
	    {
		EllipsoidModelSim ellipsoid_sim = sim.m_ellipsoid_model_sims[k];
		for(size_t l = 0; l < ellipsoid_sim.m_ellipsoid_models.size(); ++l)
		{
		    EllipsoidModel ellipsoid_model = ellipsoid_sim.m_ellipsoid_models[l];
		    // check if mu of ellipsoid model is in bbox
		    bool res = obb.checkPtInBox(ellipsoid_model.mu);
		    if (res)
			// todo: rotate ellipsoids to identity
			primitive_ellipsoid_models.push_back(ellipsoid_model);
		}

	    }
	    // debug
	    std::cout << "n ellipsoids in primitive: " << primitive_ellipsoid_models.size() << std::endl;
	    std::string rel_path_primitive_ellipsoids = 
		genRelPathPrimitiveEllipsoids(section_id, primitive_classes[i], element_ids[i][j]);
	    writeEllipsoidModelsToFile(primitive_ellipsoid_models, rel_path_primitive_ellipsoids);
	}

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}



