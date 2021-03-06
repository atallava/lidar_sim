#include <iostream>
#include <math.h>
#include <cmath>
#include <chrono>
#include <random>
#include <time.h>
#include <algorithm>

#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "eigenmvn.h"

#include <flann/flann.hpp>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    std::vector<double> calcPtsMean(const Pts &pts)
    {
	size_t pts_dim = pts[0].size();
	std::vector<double> mu(pts_dim, 0);
	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < pts_dim; ++j)
		mu[j] = mu[j]+pts[i][j];

	for(size_t i = 0; i < pts_dim; ++i)
	    mu[i] = mu[i]/pts.size();

	return mu;
    }

    Eigen::MatrixXd calcPtsCovMat(const std::vector<std::vector<double> > &pts)
    {
	Pts centered_pts = calcCenteredPts(pts);
	size_t pts_dim = pts[0].size();
	
	Eigen::MatrixXd cov_mat(pts_dim, pts_dim);
	for(size_t i = 0; i < (size_t)pts_dim; ++i)
	    for(size_t j = 0; j < (size_t)pts_dim; ++j)
		cov_mat(i,j) = 0;

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    Eigen::MatrixXd outer_prod = calcOuterProd(centered_pts[i]);
	    cov_mat = cov_mat + outer_prod;
	}

	cov_mat = cov_mat/(centered_pts.size()-1);

	return cov_mat;
    }

    Eigen::MatrixXd calcOuterProd(const std::vector<double> &pt)
    {
	size_t pt_dim = pt.size();
	Eigen::MatrixXd outer_prod(pt_dim, pt_dim);
	for(size_t i = 0; i < pt_dim; ++i)
	    for(size_t j = 0; j < pt_dim; ++j)
		outer_prod(i,j) = pt[i]*pt[j];

	return outer_prod;
    }

    Pts calcCenteredPts(const Pts &pts)
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

    bool anyNonzeros(const std::vector<int> &vec)
    {
	for(size_t i = 0; i < vec.size(); ++i)
	    if (!(vec[i] == 0))
		return true;
	return false;
    }

    std::vector<double> sampleFromMvn(const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat,
				      const bool deterministic_sampling)
    {
	Eigen::MatrixXd mean(3,1);
	for(size_t i = 0; i < 3; ++i)
	    mean(i) = mu[i];

	bool use_cholesky = false;

	uint64_t seed;
	if (deterministic_sampling)
	    seed = 1;
	else
	    seed = std::chrono::duration_cast<std::chrono::nanoseconds>
		(std::chrono::steady_clock::now().time_since_epoch()).count();

	Eigen::EigenMultivariateNormal<double> normX_solver(mean, cov_mat, use_cholesky, seed);
	Eigen::MatrixXd sample = normX_solver.samples(1);

	std::vector<double> sample_stl(3, 0);
	for(size_t i = 0; i < 3; ++i)
	    sample_stl[i] = sample(i);

	return sample_stl;
    }

    std::vector<int> negateLogicalVec(const std::vector<int> &vec)
    {
	std::vector<int> vec_negated(vec.size(), 0);
	for(size_t i = 0; i < vec.size(); ++i)
	    if (vec[i] == 0)
		vec_negated[i] = 1;

	return vec_negated;
    }

    // note: rebuilds index each time
    std::tuple<std::vector<int>, std::vector<double> >
    nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2)
    {
	std::vector<double> pts1_unrolled;
	for (size_t i = 0; i < pts1.size(); ++i)
	    pts1_unrolled.insert(pts1_unrolled.end(), pts1[i].begin(), pts1[i].end());
	flann::Matrix<double> dataset(pts1_unrolled.data(), pts1.size(), pts1[0].size());

	std::vector<double> pts2_unrolled;
	for (size_t i = 0; i < pts2.size(); ++i)
	    pts2_unrolled.insert(pts2_unrolled.end(), pts2[i].begin(), pts2[i].end());
	flann::Matrix<double> query(pts2_unrolled.data(), pts2.size(), pts2[0].size());

	int nn = 1;

	std::vector<std::vector<int> > indices;
	std::vector<std::vector<double> > dists;
    
	// randomized kd trees
	// int n_kd_trees = 10; 
	// flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(n_kd_trees));

	// hierarchical k-means
	// the parameters values are mostly default
	// using pp centers for repeatability
	flann::Index<flann::L2<double> > index(dataset, 
					       flann::KMeansIndexParams(32, 11, 
									flann::FLANN_CENTERS_KMEANSPP));
	index.buildIndex();                                                                                               
	int n_checks = 100; 
	index.knnSearch(query, indices, dists, nn, flann::SearchParams(n_checks));

	std::vector<int> ids(pts2.size(), 0);
	std::vector<double> nearest_dists(pts2.size(), 0);
	for(size_t i = 0; i < pts2.size(); ++i)
	{
	    ids[i] = indices[i][0];
	    // flann returns squared distances!!
	    nearest_dists[i] = std::sqrt(dists[i][0]);
	}

	return std::make_tuple(ids, nearest_dists);
    }

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
    nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2, const int nn)
    {
	std::vector<double> pts1_unrolled;
	for (size_t i = 0; i < pts1.size(); ++i)
	    pts1_unrolled.insert(pts1_unrolled.end(), pts1[i].begin(), pts1[i].end());
	flann::Matrix<double> dataset(pts1_unrolled.data(), pts1.size(), pts1[0].size());

	std::vector<double> pts2_unrolled;
	for (size_t i = 0; i < pts2.size(); ++i)
	    pts2_unrolled.insert(pts2_unrolled.end(), pts2[i].begin(), pts2[i].end());
	flann::Matrix<double> query(pts2_unrolled.data(), pts2.size(), pts2[0].size());

	std::vector<std::vector<int> > indices;
	std::vector<std::vector<double> > dists;
    
	// randomized kd trees
	// int n_kd_trees = 10; 
	// flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(n_kd_trees));

	// hierarchical k-means
	// the parameters values are mostly default
	// using pp centers for repeatability
	flann::Index<flann::L2<double> > index(dataset, 
					       flann::KMeansIndexParams(32, 11, 
									flann::FLANN_CENTERS_KMEANSPP));
	index.buildIndex();
	
	int n_checks = 100;
	index.knnSearch(query, indices, dists, nn, flann::SearchParams(n_checks));

	// flann returns squared distances
	for(size_t i = 0; i < dists.size(); ++i)
	    for(size_t j = 0 ; j < dists[i].size(); ++j)
		dists[i][j] = std::sqrt(dists[i][j]);
	
	return std::make_tuple(indices, dists);
    }

    std::vector<std::vector<double> > pdist2(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2)
    {
	size_t n_pts1 = pts1.size();
	size_t n_pts2 = pts2.size();
	std::vector<std::vector<double> > D(n_pts1, std::vector<double>(n_pts2));

	for(size_t i = 0; i < n_pts1; ++i)
	    for(size_t j = 0; j < n_pts2; ++j)
		D[i][j] = euclideanDist(pts1[i], pts2[j]);

	return D;
    }

    std::tuple<double, double, double> cart2sph(std::vector<double> vec)
    {
	double x = vec[0]; double y = vec[1]; double z = vec[2];

	double theta = std::atan2(y, x);
	double phi = std::atan2(z, 
				std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0)));
	double r = vectorNorm(vec);

	return std::make_tuple(theta, phi, r);
    }

    std::vector<double> linspace(const double x, const double y, const int n)
    {
	double spacing = (y-x)/(n-1);
	std::vector<double> vec(n, 0);
	for (size_t i = 0; i < (size_t)n; ++i)
	    vec[i] = x + i*spacing;

	return vec;
    }

    double euclideanDist(const std::vector<double> &pt1, const std::vector<double> &pt2)
    {
	double dist = 0;
	for(size_t i = 0; i < pt1.size(); ++i)
	{
	    try
	    {
		dist = dist + std::pow(pt1[i]-pt2[i], 2);
	    }
	    catch (const std::exception& e)
	    {
		std::cout << "error. something bad happened" << std::endl;
		exit(0);
	    }
	}

	dist = std::sqrt(dist);
	return dist;
    }

    double calcVariance(const std::vector<double> &vec)
    {
	double mu = 0;
	for(size_t i = 0; i < vec.size(); ++i)
	    mu += vec[i];
	mu /= vec.size();

	double var = 0;
	for(size_t i = 0; i < vec.size(); ++i)
	    var += std::pow((vec[i] - mu), 2);
	var /= vec.size();

	return var;
    }

    std::vector<int> genLogicalVecFromIds(std::vector<int> ids, int vec_size)
    {
	std::vector<int> vec(vec_size, 0);
	for(auto i : ids)
	    vec[i] = 1;

	return vec;
    }

    double vectorNorm(const std::vector<double> &a)
    {
	double res = 0;
	for(size_t i = 0; i < a.size(); ++i)
	    res += std::pow(a[i], 2.0);
	res = std::sqrt(res);

	return res;
    }

    double dotProduct(const std::vector<double> &a, const std::vector<double> &b)
    {
	double res = 0;
	for(size_t i = 0; i < a.size(); ++i)
	    res += a[i]*b[i];

	return res;
    }

    std::vector<double> vectorDiff(const std::vector<double> &a, const std::vector<double> &b)
    {
	std::vector<double> c(3, 0.0);
	for(size_t i = 0; i < a.size(); ++i)
	    c[i] = a[i]-b[i];

	return c;
    }

    std::vector<double> getPerpUnitVec2(const std::vector<double> &a)
    {
	std::vector<double> b(2, 0);
	b[0] = -a[1];
	b[1] = a[0];

	return normalizeVec(b);
    }
    
    std::vector<double> normalizeVec(const std::vector<double> &a)
    {
	double norm = vectorNorm(a);
	std::vector<double> b(a.size(), 0);
	for(size_t i = 0; i < a.size(); ++i)
	    b[i] = a[i]/norm;

	return b;
    }

    std::vector<std::vector<double> > calcPrincipalAxes2D(const std::vector<std::vector<double> > &pts)
    {
	Eigen::MatrixXd cov_mat = calcPtsCovMat(pts);
	Eigen::EigenSolver<Eigen::MatrixXd> es(cov_mat);
	Eigen::MatrixXcd V = es.eigenvectors();

	std::vector<std::vector<double> > eigenvectors(2, std::vector<double>(2, 0));
	for(size_t i = 0; i < 2; ++i)
	    for(size_t j = 0; j < 2; ++j)
		eigenvectors[i][j] = std::real(V(i,j));

	return eigenvectors;
    }

    OrientedBox
    calcObb(const std::vector<std::vector<double> > &pts)
    {
	OrientedBox obb;
	std::vector<std::vector<double> > xy(pts.size(), std::vector<double>(2, 0));
	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < 2; ++j)
		xy[i][j] = pts[i][j];

	obb.m_center = calcPtsMean(xy);
	obb.m_axes = calcPrincipalAxes2D(xy);
	obb.m_intervals = std::vector<std::vector<double> > {{0, 0}, {0, 0}};

	for(size_t i = 0; i < xy.size(); ++i)
	{
	    std::vector<double> m_centered_pt(2, 0);
	    for(size_t j = 0; j < 2; ++j)
		m_centered_pt[j] = xy[i][j] - obb.m_center[j];
	
	    for(size_t j = 0; j < 2; ++j)
	    {
		double projn = dotProduct(m_centered_pt, obb.m_axes[j]);
		if (projn < obb.m_intervals[j][0])
		    obb.m_intervals[j][0] = projn;
		if (projn > obb.m_intervals[j][1])
		    obb.m_intervals[j][1] = projn;
	    }
	}

	obb.padIntervals();
	obb.calcVertices();
	return obb;
    }

    std::vector<std::vector<double> > centerPts(const std::vector<std::vector<double> > &pts)
    {
	size_t pts_dim = pts[0].size();
	std::vector<double> mu = calcPtsMean(pts);
	std::vector<std::vector<double> > centered_pts(pts.size(), std::vector<double>(pts_dim, 0));
	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < pts_dim; ++j)
		centered_pts[i][j] = pts[i][j] - mu[j];

	return centered_pts;
    }

    Eigen::Matrix3d rotz(const double theta)
    {
	Eigen::Matrix3d R;
	R << 
	    std::cos(theta), -std::sin(theta), 0,
	    std::sin(theta), std::cos(theta), 0, 
	    0, 0, 1;

	return R;
    }

    Eigen::Matrix4d transfz(const std::vector<double> xyz, const double theta)
    {
	Eigen::Matrix4d T;
	T << 
	    std::cos(theta), -std::sin(theta), 0, xyz[0],
	    std::sin(theta), std::cos(theta), 0, xyz[1],
	    0, 0, 1, xyz[2],
	    0, 0, 0, 1;

	return T;
    }

    std::vector<std::vector<double> > applyTransfToPts(std::vector<std::vector<double> > pts_1, 
						       const Eigen::Matrix4d &T_1_to_2)
    {
	bool homogeneous_input;
	if (pts_1[0].size() == 3)
	{
	    homogeneous_input = false;
	    for(size_t i = 0; i < pts_1.size(); ++i)
		pts_1[i].push_back(1);
	}
	else
	    homogeneous_input = true;

	Eigen::MatrixXd pts_1_eigen = stlArrayToEigen(pts_1);
	Eigen::MatrixXd pts_2_eigen = T_1_to_2*pts_1_eigen.transpose();
	std::vector<std::vector<double> > pts_2 = EigenToStlArray(pts_2_eigen.transpose());
	
	if (!homogeneous_input)
	    for(size_t i = 0; i < pts_2.size(); ++i)
		pts_2[i].erase(pts_2[i].end()-1);

	return pts_2;
    }

    void GetEllipseTransform(const Eigen::Matrix3d &input, Eigen::Quaterniond &quat, 
			     Eigen::Vector3d &scale, double level)
    {
	Eigen::Matrix3d levelled_input(input*level);
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(levelled_input.inverse(), Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Vector3d singular_values = svd.singularValues();

	quat = Eigen::Quaterniond(U);

	// scales
	for(size_t i = 0; i < 3; ++i)
	    scale(i) = 1/std::sqrt(singular_values(i));

	// debug
	// std::cout << "U: " << U << std::endl;
	// std::cout << "sv: " << singular_values << std::endl;
	// std::cout << "quat weight: " << quat.w() << std::endl;
	// std::cout << "quat vec: " << quat.vec() << std::endl;
	// std::cout << "scales: " << scale << std::endl;
    }

    double calcF1Score(double precision, double recall)
    {
	double f1_score = 2*(precision*recall)/(precision + recall);
	return f1_score;
    }

    int getNumDigits(int x)
    {
	return (x < 10 ? 1 :   
		(x < 100 ? 2 :   
		 (x < 1000 ? 3 :   
		  (x < 10000 ? 4 :   
		   (x < 100000 ? 5 :   
		    (x < 1000000 ? 6 :   
		     (x < 10000000 ? 7 :  
		      (x < 100000000 ? 8 :  
		       (x < 1000000000 ? 9 :  
			10)))))))));  
    }
}

