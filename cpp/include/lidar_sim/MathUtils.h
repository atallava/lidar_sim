#pragma once
#include <vector>
#include <string>

#include <Eigen/Dense>

namespace lidar_sim {
    typedef std::vector<std::vector<double> > Pts;

    std::vector<double> calcPtsMean(std::vector<std::vector<double> > pts);
    Pts calcCenteredPts(Pts pts);
    Eigen::MatrixXd calcPtsCovMat(std::vector<std::vector<double> > pts);
    Eigen::MatrixXd calcOuterProd(std::vector<double> pt);
    double deg2rad(double angle_deg);
    bool anyNonzeros(std::vector<int> vec);
    std::vector<double> sampleFromMvn(std::vector<double> mu, Eigen::MatrixXd cov_mat);
}

