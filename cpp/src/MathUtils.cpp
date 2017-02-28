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

    std::vector<double> sampleFromMvn(const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat)
    {
	Eigen::MatrixXd mean(3,1);
	for(size_t i = 0; i < 3; ++i)
	    mean(i) = mu[i];

	bool use_cholesky = false;
	// should the seed be passed as argument?
	// what a long-winded seed. had to go to level o
	uint64_t seed = std::chrono::duration_cast<std::chrono::nanoseconds>
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

    std::tuple<std::vector<int>, std::vector<double> >
    nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2)
    {
	flann::Matrix<double> dataset = stlArrayToFlannMatrix(pts1);
	flann::Matrix<double> query = stlArrayToFlannMatrix(pts2);

	int nn = 1;

	flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
	flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);

	int n_kd_trees = 10;
	flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(n_kd_trees));
	index.buildIndex();                                                                                               
	int n_checks = 100;
	index.knnSearch(query, indices, dists, nn, flann::SearchParams(n_checks));

	std::vector<int> ids(pts2.size(), 0);
	std::vector<double> nearest_dists(pts2.size(), 0);
	for(size_t i = 0; i < pts2.size(); ++i)
	{
	    ids[i] = indices[i][0];
	    nearest_dists[i] = dists[i][0];
	}

	return std::make_tuple(ids, nearest_dists);
    }

    std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
    nearestNeighbors(const std::vector<std::vector<double> > &pts1, const std::vector<std::vector<double> > &pts2, const int nn)
    {
	flann::Matrix<double> dataset = stlArrayToFlannMatrix(pts1);
	flann::Matrix<double> query = stlArrayToFlannMatrix(pts2);

	flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
	flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);

	int n_kd_trees = 10;
	flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(n_kd_trees));
	index.buildIndex();                                                                                               
	int n_checks = 100;
	index.knnSearch(query, indices, dists, nn, flann::SearchParams(n_checks));
	
	return std::make_tuple(
	    flannMatrixToStlArray(indices), flannMatrixToStlArray(dists));
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

    double euclideanDist(const std::vector<double> &pt1, const std::vector<double> &pt2)
    {
	double dist = 0;
	for(size_t i = 0; i < pt1.size(); ++i)
	    dist = dist + std::pow(pt1[i]-pt2[i], 2);

	dist = std::sqrt(dist);
	return dist;
    }

    std::tuple<std::vector<int>, std::vector<double> >
    sortIntersectionFlag(const std::vector<int> &intersection_flag, const std::vector<double> &dist_along_ray)
    {
	std::vector<int> intersecting_ids;
	std::vector<double> dist_along_ray_intersections;
	for(size_t i = 0; i < intersection_flag.size(); ++i)
	    if (intersection_flag[i] == 1)
	    {
		intersecting_ids.push_back(i);
		dist_along_ray_intersections.push_back(dist_along_ray[i]);
	    }
    
	// get indices of ascending sort
	std::vector<int> sorted_ids(intersecting_ids.size());
	std::size_t n(0);
	std::generate(std::begin(sorted_ids), std::end(sorted_ids), [&]{ return n++; });

	std::sort( std::begin(sorted_ids), std::end(sorted_ids), 
		   [&](int i1, int i2) { return dist_along_ray_intersections[i1] < dist_along_ray_intersections[i2]; });

	std::vector<int> sorted_intersecting_ids(intersecting_ids.size());
	std::vector<double> sorted_dist_along_ray_intersections(dist_along_ray_intersections.size());
	for(size_t i = 0; i < sorted_ids.size(); ++i)
	{
	    sorted_intersecting_ids[i] = intersecting_ids[sorted_ids[i]];
	    sorted_dist_along_ray_intersections[i] = dist_along_ray_intersections[sorted_ids[i]];
	}
    
	return std::make_tuple(sorted_intersecting_ids, sorted_dist_along_ray_intersections);
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

    std::tuple<int, bool>
    sampleHitId(const std::vector<double> &hit_prob_vec, const std::vector<int> &target_ids)
    {
	size_t n_targets = hit_prob_vec.size();
	std::random_device rd;
	std::mt19937 gen(rd());
   	std::uniform_real_distribution<> dis(0, 1);

	int hit_id;
	bool hit_bool;
	for(size_t i = 0; i < n_targets; ++i)
	    if (dis(gen) < hit_prob_vec[i])
	    {
		hit_id = target_ids[i];
		hit_bool = true;
		return std::make_tuple(hit_id, hit_bool);
	    }
	hit_id = -1;
	hit_bool = false;
	return std::make_tuple(hit_id, hit_bool);
    }

    std::vector<int> getIntersectedFlag(const std::vector<std::vector<int> > &intersection_flag)
    {
	std::vector<int> intersected_flag(intersection_flag[0].size(), 0);
	for(size_t i = 0; i < intersection_flag[0].size(); ++i)
	    for(size_t j = 0; j < intersection_flag.size(); ++j)
		intersected_flag[i] += intersection_flag[j][i];

	return intersected_flag;
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
}

