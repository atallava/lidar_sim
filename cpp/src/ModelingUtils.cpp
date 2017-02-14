#include <iostream>
#include <sstream>

#include <lidar_sim/ModelingUtils.h>

using namespace lidar_sim;

namespace lidar_sim {
    EllipsoidModel createEllipsoidModel(Pts pts)
    {
	EllipsoidModel model;
	model.mu = calcPtsMean(pts);
	model.cov_mat = calcPtsCovMat(pts);
	model.hit_prob = 1;

	return model;
    }

    void writeEllipsoidModelsToFile(EllipsoidModels ellipsoid_models, std::string rel_path_output)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing ellipsoid models to: " << rel_path_output << std::endl;
	
	for(size_t i = 0; i < ellipsoid_models.size(); ++i)
	{
	    // mu
	    std::vector<double> mu = ellipsoid_models[i].mu;
	    std::ostringstream line;
	    for(size_t j = 0; j < 3; ++j)
		line << mu[j] << " ";

	    // cov mat
	    Eigen::MatrixXd cov_mat = ellipsoid_models[i].cov_mat;
	    for(size_t j = 0; j < 3; ++j)
		for(size_t k = 0; k < 3; ++k)
		    line << cov_mat(k,j) << " ";
	    
	    // hit_prob
	    double hit_prob = ellipsoid_models[i].hit_prob;
	    line << hit_prob << std::endl;
	    
	    file << line.str();
	}

	file.close();
    }

    EllipsoidModels loadEllipsoidModelsFromFile(std::string rel_path_input)
    {
	// open input file
	std::ifstream file(rel_path_input);
	std::cout << "Reading ellipsoid models from: " << rel_path_input << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	EllipsoidModels ellipsoid_models;

	std::string current_line;
	while(std::getline(file, current_line))
	{
	    std::istringstream iss(current_line);

	    std::vector<double> mu(3,0);
	    for(size_t i = 0; i < 3; ++i)
		iss >> mu[i];

	    Eigen::MatrixXd cov_mat(3,3);
	    for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 3; ++j)
		    iss >> cov_mat(j,i);

	    double hit_prob;
	    iss >> hit_prob;

	    EllipsoidModel ellipsoid_model;
	    ellipsoid_model.mu = mu;
	    ellipsoid_model.cov_mat = cov_mat;
	    ellipsoid_model.hit_prob = hit_prob;

	    ellipsoid_models.push_back(ellipsoid_model);
	}

	file.close();

	return ellipsoid_models;
    }

    void dispEllipsoidModel(EllipsoidModel model)
    {
	std::cout << "mu: " << std::endl;
	for(size_t i = 0; i < 3; ++i)
	    std::cout << model.mu[i] << " ";
	std::cout << std::endl;

	std::cout << "cov_mat: " << std::endl;
	std::cout << model.cov_mat << std::endl;

	std::cout << "prob_hit: " << std::endl;
	std::cout << model.hit_prob << std::endl;
    }

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<int> > > 
    buildBlocks(const std::vector<std::vector<double> > &imu_posn_nodes,
		const std::vector<std::vector<double> > &pts, int pts_per_block)
    {
	// nearest neighbor for pts in nodes
	std::vector<std::vector<int> > nn_ids;
	std::vector<std::vector<double> > nn_dists;
	std::tie(nn_ids, nn_dists) = nearestNeighbors(imu_posn_nodes, pts, 1);
    
	// for each node, which pts are in it
	std::vector<std::vector<int> > node_pts_map(imu_posn_nodes.size(), 
						    std::vector<int> ());
    
	for(size_t i = 0; i < pts.size(); ++i)
	    node_pts_map[nn_ids[i][0]].push_back(i);
    
	// block node ids
	std::vector<std::vector<int> > block_node_ids;
	std::vector<int> current_block_node_ids(2, 0);
	current_block_node_ids[0] = 0;

	int block_id = 1;

	int pts_in_current_block = 0;
	for(size_t i = 0; i < node_pts_map.size(); ++i)
	{
	    pts_in_current_block += node_pts_map[i].size();

	    if (pts_in_current_block >= pts_per_block)
	    {
		if (i == node_pts_map.size()-1)
		    break;

		// block node ids
		current_block_node_ids[1] = i;
		block_node_ids.push_back(current_block_node_ids);
		current_block_node_ids[0] = i+1;

		// increment block id
		block_id++;

		// reset counter
		pts_in_current_block = 0;
	    }
	}
	current_block_node_ids[1] = node_pts_map.size()-1;
	block_node_ids.push_back(current_block_node_ids);

	
	// pts ids in each block
	std::vector<std::vector<int> > block_pts_ids(block_node_ids.size(), std::vector<int>());
	for(size_t i = 0; i < block_node_ids.size(); ++i)
	    for(size_t j = block_node_ids[i][0]; j <= (size_t)block_node_ids[i][1]; ++j)
		block_pts_ids[i].insert(block_pts_ids[i].end(), node_pts_map[j].begin(), node_pts_map[j].end());

	return std::make_tuple(block_node_ids, block_pts_ids);
    }
}
