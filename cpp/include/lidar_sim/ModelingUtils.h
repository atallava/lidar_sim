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

    struct EllipsoidModel;
    
    EllipsoidModel createEllipsoidModel(Pts pts);
    void writeEllipsoidModelsToFile(EllipsoidModels ellipsoid_models, std::string rel_path_output);
    EllipsoidModels loadEllipsoidModelsFromFile(std::string rel_path_input);
    void dispEllipsoidModel(EllipsoidModel model);

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<int> > >
	buildBlocks(const std::vector<std::vector<double> > &imu_posn_nodes,
		    const std::vector<std::vector<double> > &pts, int pts_per_block);
}

