#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {

    struct EllipsoidModel {
	std::vector<double> mu;
	Eigen::MatrixXd cov_mat;
	double perm;
    };

    typedef std::vector<EllipsoidModel> EllipsoidModels;

    typedef std::vector<std::vector<double> > Pts;

    std::vector<double> calcPtsMean(std::vector<std::vector<double> > pts);
    Pts calcCenteredPts(Pts pts);
    Eigen::MatrixXd calcPtsCovMat(std::vector<std::vector<double> > pts);
    Eigen::MatrixXd calcOuterProd(std::vector<double> pt);
    EllipsoidModel createEllipsoidModel(Pts pts);
    void dispPt(std::vector<double> pt);
}

