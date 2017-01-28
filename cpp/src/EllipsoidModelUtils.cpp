#include <iostream>
#include <sstream>

#include <lidar_sim/EllipsoidModelUtils.h>

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

    EllipsoidModels loadEllipsoidModels(std::string rel_path_input)
    {
	// open input file
	std::ifstream file(rel_path_input);
	std::cout << "Reading ellipsoid models from: " << rel_path_input << std::endl;

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

    std::vector<int> getIntersectedFlag(std::vector<std::vector<int> > intersection_flag)
    {
	std::vector<int> intersected_flag(intersection_flag[0].size(), 0);
	for(size_t i = 0; i < intersection_flag[0].size(); ++i)
	    for(size_t j = 0; j < intersection_flag.size(); ++j)
		intersected_flag[i] += intersection_flag[j][i];

	return intersected_flag;
    }
}
