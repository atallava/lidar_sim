#include <string>
#include <iostream>
#include <vector>
#include <ctime>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/SceneObjectServer.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

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

std::string genRelPathBlockEllipsoidModels(int section_id, int block_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground_ellipsoids.txt";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    // annotations
    std::string rel_path_annotations = "data/sections/section_04/section_04_annotations.txt";
    std::vector<std::string> object_classes;
    std::vector<std::vector<double> > object_xy_posns;
    std::tie(object_classes, object_xy_posns) = 
	loadAnnotations(rel_path_annotations);

    // imu posn nodes
    int section_id = 4;
    std::vector<std::vector<double> > imu_posn_nodes = loadArray(genRelPathImuPosnNodes(section_id), 3);

    // add proxy z to object posns
    std::vector<double> imu_posn_nodes_mu = calcPtsMean(imu_posn_nodes);
    double z_proxy = imu_posn_nodes_mu[2];
    std::vector<std::vector<double> > object_posns(object_xy_posns.size(), std::vector<double>(3, 0));
    for(size_t i = 0; i < object_xy_posns.size(); ++i)
    {
	object_posns[i][0] = object_xy_posns[i][0];
	object_posns[i][1] = object_xy_posns[i][1];
	object_posns[i][2] = z_proxy;
    }

    // create non ground blocks
    int num_objects_per_block = 10;
    std::vector<std::vector<int> > block_node_ids;
    std::vector<std::vector<int> > block_object_ids;
    std::tie(block_node_ids, block_object_ids) = 
	buildBlocks(imu_posn_nodes, object_posns, num_objects_per_block);

    // write out non ground block ids
    writePtsToXYZFile(block_node_ids, genRelPathBlockNodeIdsNonGround(section_id));

    // build block ellipsoid models
    SceneObjectServer object_server;
    std::string rel_path_primitive_ellipsoids_dir = "data/sections/section_03/primitives/ellipsoids";
    object_server.loadPrimitiveEllipsoidModels(rel_path_primitive_ellipsoids_dir);

    for(size_t i = 0; i < block_object_ids.size(); ++i)
    {
	EllipsoidModels ellipsoid_models;
	std::vector<int> this_block_object_ids = block_object_ids[i];
	for(size_t j = 0; j < this_block_object_ids.size(); ++j)
	{
	    EllipsoidModels object_ellipsoid_models;
	    int object_id = this_block_object_ids[j];
	    // todo: currently not taking into account orientation of primitive
	    object_ellipsoid_models = 
		object_server.genObjectModelsAtPosn(object_classes[object_id], object_posns[object_id]);
	    ellipsoid_models.insert(ellipsoid_models.end(), object_ellipsoid_models.begin(), object_ellipsoid_models.end());
	}

	// write this block ellipsoid models
	int block_id = i + 1;
	writeEllipsoidModelsToFile(ellipsoid_models,
				   genRelPathBlockEllipsoidModels(section_id, block_id));
    }

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;

    return(1);
}


