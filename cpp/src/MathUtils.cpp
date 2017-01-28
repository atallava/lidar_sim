#include <iostream>
#include <math.h>

#include <lidar_sim/MathUtils.h>

namespace lidar_sim {
    std::vector<double> calcPtsMean(Pts pts)
    {
	std::vector<double> mu(3,0);
	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < 3; ++j)
		mu[j] = mu[j]+pts[i][j];

	for(size_t i = 0; i < 3; ++i)
	    mu[i] = mu[i]/pts.size();

	return mu;
    }

    Eigen::MatrixXd calcPtsCovMat(std::vector<std::vector<double> > pts)
    {
	Pts centered_pts = calcCenteredPts(pts);
	
	Eigen::MatrixXd cov_mat(3,3);
	for(size_t i = 0; i < 3; ++i)
	    for(size_t j = 0; j < 3; ++j)
		cov_mat(i,j) = 0;

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    Eigen::MatrixXd outer_prod = calcOuterProd(centered_pts[i]);
	    cov_mat = cov_mat + outer_prod;
	}

	cov_mat = cov_mat/(centered_pts.size()-1);

	return cov_mat;
    }

    Eigen::MatrixXd calcOuterProd(std::vector<double> pt)
    {
	Eigen::MatrixXd outer_prod(3,3);
	for(size_t i = 0; i < 3; ++i)
	    for(size_t j = 0; j < 3; ++j)
		outer_prod(i,j) = pt[i]*pt[j];

	return outer_prod;
    }

    Pts calcCenteredPts(Pts pts)
    {
	std::vector<double> mu = calcPtsMean(pts);

	Pts centered_pts; 
	centered_pts = pts;

	// subtract mean from pts
	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < 3; ++j)
		centered_pts[i][j] = pts[i][j]-mu[j];

	return centered_pts;
    }

    double deg2rad(double angle_deg)
    {
	return angle_deg*M_PI/180;
    }
}
