#include <iostream>
#include <math.h>
#include <cmath>
#include <algorithm>

#include "eigenmvn.h"

#include <flann/flann.hpp>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>

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

    bool anyNonzeros(std::vector<int> vec)
    {
	for(size_t i = 0; i < vec.size(); ++i)
	    if (!(vec[i] == 0))
		return true;
	return false;
    }

    std::vector<double> sampleFromMvn(std::vector<double> mu, Eigen::MatrixXd cov_mat)
    {
	Eigen::MatrixXd mean(3,1);
	for(size_t i = 0; i < 3; ++i)
	    mean(i) = mu[i];

	Eigen::EigenMultivariateNormal<double> normX_solver(mean, cov_mat);
	Eigen::MatrixXd sample = normX_solver.samples(1);

	std::vector<double> sample_stl(3, 0);
	for(size_t i = 0; i < 3; ++i)
	    sample_stl[i] = sample(i);

	return sample_stl;
    }

    std::vector<int> nearestNeighbors(std::vector<std::vector<double> > pts1, std::vector<std::vector<double> > pts2)
    {
	flann::Matrix<double> dataset = stlArrayToFlannMatrix(pts1);
	flann::Matrix<double> query = stlArrayToFlannMatrix(pts2);

	int nn = 1;

	flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
	flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);

	// construct an randomized kd-tree index using 4 kd-trees
	int n_kd_trees = 4;
	flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(n_kd_trees));
	index.buildIndex();                                                                                               
	int n_checks = 20;
	index.knnSearch(query, indices, dists, nn, flann::SearchParams(n_checks));

	std::vector<int> ids(pts2.size(), 0);
	for(size_t i = 0; i < pts2.size(); ++i)
	    ids[i] = indices[i][0];

	return ids;
    }

    std::vector<std::vector<double> > pdist2(std::vector<std::vector<double> > pts1, std::vector<std::vector<double> > pts2)
    {
	size_t n_pts1 = pts1.size();
	size_t n_pts2 = pts2.size();
	std::vector<std::vector<double> > D(n_pts1, std::vector<double>(n_pts2));

	for(size_t i = 0; i < n_pts1; ++i)
	    for(size_t j = 0; j < n_pts2; ++j)
		D[i][j] = euclideanDist(pts1[i], pts2[j]);

	return D;
    }

    double euclideanDist(std::vector<double> pt1, std::vector<double> pt2)
    {
	double dist = 0;
	for(size_t i = 0; i < pt1.size(); ++i)
	    dist = dist + std::pow(pt1[i]-pt2[i], 2);

	dist = std::sqrt(dist);
	return dist;
    }
 }
