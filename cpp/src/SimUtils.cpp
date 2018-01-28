#include <lidar_sim/SimUtils.h>
#include <lidar_sim/DataProcessingUtils.h>

namespace lidar_sim {
    SectionModelSim createSectionModelSimObject(int section_models_id, std::string sim_version, bool deterministic_sim)
    {
	std::string path_models_dir = genPathHgModelsDir(section_models_id, sim_version);

	// find ellipsoid models for this section
	std::vector<std::string> rel_path_ellipsoid_model_blocks;
	std::vector<int> ellipsoid_model_block_ids = 
	    getEllipsoidModelBlockIds(path_models_dir, section_models_id);
	for(auto i : ellipsoid_model_block_ids)
	    rel_path_ellipsoid_model_blocks.push_back(genRelPathEllipsoids(section_models_id, sim_version, i));

	// find triangle models for this section
	std::vector<std::string> rel_path_triangle_model_blocks;
	std::vector<int> triangle_model_block_ids = 
	    getTriangleModelBlockIds(path_models_dir, section_models_id);
	for(auto i : triangle_model_block_ids)
	    rel_path_triangle_model_blocks.push_back(genRelPathTriangles(section_models_id, sim_version, i));

	// create sim object
	SectionModelSim sim;
	sim.loadEllipsoidModelBlocks(rel_path_ellipsoid_model_blocks);
	sim.loadTriangleModelBlocks(rel_path_triangle_model_blocks);

	sim.setDeterministicSim(deterministic_sim);

	std::string path_imu_posn_nodes = genPathImuPosnNodes(section_models_id);
	std::string path_block_node_ids_ground = genPathBlockNodeIdsGround(section_models_id);
	std::string path_block_node_ids_non_ground = genPathBlockNodeIdsNonGround(section_models_id);

	sim.loadBlockInfo(path_imu_posn_nodes, path_block_node_ids_ground, path_block_node_ids_non_ground);

	return sim;
    }
}

