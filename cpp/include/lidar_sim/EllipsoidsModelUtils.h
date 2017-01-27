#pragma once
#include <vector>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include <lidar_sim/MathUtils.h>

namespace lidar_sim {

    struct EllipsoidModel {
	std::vector<double> mu;
	Eigen::MatrixXd cov_mat;
	double hit_prob;
    };

    typedef std::vector<EllipsoidModel> EllipsoidModels;

    EllipsoidModel createEllipsoidModel(Pts pts);
    void writeEllipsoidModelsToFile(EllipsoidModels ellipsoid_models, std::string rel_path_output);
    EllipsoidModels loadEllipsoidModels(std::string rel_path_input);
}

