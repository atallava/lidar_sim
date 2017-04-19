#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/LaserUtils.h>

using namespace lidar_sim;

EllipsoidModelSim::EllipsoidModelSim() :
    m_max_maha_dist_for_hit(3.5),
    m_debug_flag(0),
    m_deterministic_sim(false)
{    
    std::random_device rd;
    std::mt19937 gen(rd());
    m_gen = gen;
}

void EllipsoidModelSim::setEllipsoidModels(const EllipsoidModels &ellipsoid_models)
{
    m_ellipsoid_models = ellipsoid_models;
}

void EllipsoidModelSim::loadEllipsoidModels(const std::string rel_path_models)
{
    m_ellipsoid_models = loadEllipsoidModelsFromFile(rel_path_models);
}

void EllipsoidModelSim::setLaserCalibParams(const LaserCalibParams laser_calib_params)
{
    m_laser_calib_params = laser_calib_params;
}

void EllipsoidModelSim::setDebugFlag(const int value)
{
    m_debug_flag = value;
}

std::tuple<std::vector<std::vector<int> >,
	   std::vector<std::vector<double> > > EllipsoidModelSim::calcEllipsoidIntersections(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns)
{
    size_t n_ellipsoids = m_ellipsoid_models.size();
    
    // debug
    if (m_debug_flag)
	n_ellipsoids = 1;

    size_t n_rays = ray_dirns.size();
    std::vector<std::vector<double> > maha_dist_to_mu(n_rays, std::vector<double>(n_ellipsoids));
    std::vector<std::vector<double> > dist_along_ray(n_rays, std::vector<double>(n_ellipsoids));
    
    for(size_t i = 0; i < n_rays; ++i)
    {
	std::vector<double> ray_dirn = ray_dirns[i];
	for(size_t j = 0; j < n_ellipsoids; ++j)
	{
	    std::vector<double> mu = m_ellipsoid_models[j].mu;
	    Eigen::MatrixXd cov_mat = m_ellipsoid_models[j].cov_mat;
	    
	    double this_dist_along_ray, this_maha_dist;
	    std::tie(this_maha_dist, this_dist_along_ray) = calcMahaDistRayToEllipsoid(ray_origin, ray_dirn, mu, cov_mat);
	    maha_dist_to_mu[i][j] = this_maha_dist;
	    dist_along_ray[i][j] = this_dist_along_ray;

	    // debug
	    if (m_debug_flag)
	    {
		std::cout << "mu, cov_mat: " << std::endl;
		dispVec(mu); 
		std::cout << cov_mat << std::endl;
		std::cout << "this maha dist, dist along ray: " << this_maha_dist << " " << this_dist_along_ray << std::endl;
	    } 
	}
    }

    std::vector<std::vector<int> > intersection_flag(n_rays, std::vector<int>(n_ellipsoids));
    for(size_t i = 0; i < n_rays; ++i)
	for(size_t j = 0; j < n_ellipsoids; ++j)
	    if ((maha_dist_to_mu[i][j] <= m_max_maha_dist_for_hit) && 
		(dist_along_ray[i][j] <= m_laser_calib_params.intrinsics.max_range) &&
		(dist_along_ray[i][j] >= m_laser_calib_params.intrinsics.min_range))
		intersection_flag[i][j] = 1;
	    else
		intersection_flag[i][j] = 0;
    
    return std::make_tuple(intersection_flag, dist_along_ray);
}

std::tuple<std::vector<int>,
	   std::vector<double> > EllipsoidModelSim::calcEllipsoidIntersections(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn)
{
    std::vector<std::vector<double> > ray_dirns;
    ray_dirns.push_back(ray_dirn);

    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = calcEllipsoidIntersections(
    	ray_origin, ray_dirns);

    return std::make_tuple(intersection_flag[0], dist_along_ray[0]);
}

std::tuple<double, double> EllipsoidModelSim::calcMahaDistRayToEllipsoid(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn, const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat)
{
    // conversion between types
    Eigen::MatrixXd v(3,1);
    for(size_t i = 0; i < 3; ++i)
	v(i) = mu[i] - ray_origin[i];

    Eigen::MatrixXd ray_dirn_eigen(3,1);
    for(size_t i = 0; i < 3; ++i)
	ray_dirn_eigen(i) = ray_dirn[i];

    double t_num = (ray_dirn_eigen.transpose()*(cov_mat.inverse()*v))(0);
    double t_denom = (ray_dirn_eigen.transpose()*(cov_mat.inverse()*ray_dirn_eigen))(0);
    double dist_along_ray = t_num/t_denom;

    Eigen::MatrixXd ray_origin_eigen(3,1);
    for(size_t i = 0; i < 3; ++i)
	ray_origin_eigen(i) = ray_origin[i];

    Eigen::MatrixXd mu_eigen(3,1);
    for(size_t i = 0; i < 3; ++i)
	mu_eigen(i) = mu[i];
    
    Eigen::MatrixXd q(3,1);
    q = ray_origin_eigen + dist_along_ray*ray_dirn_eigen;
    double maha_dist_to_mu = ((q-mu_eigen).transpose()*(cov_mat.inverse()*(q-mu_eigen)))(0);
    maha_dist_to_mu = std::sqrt(maha_dist_to_mu);

    // debug
    if (m_debug_flag)
    {
	std::cout << "ray origin: " << std::endl;
	dispVec(ray_origin);
	std::cout << "ray dirn: " << std::endl;
	dispVec(ray_dirn);
	std::cout << "ray dirn eigen: " << ray_dirn_eigen << std::endl;
	std::cout << "cov mat inv: " << cov_mat.inverse() << std::endl;
	std::cout << "cov mat inv*ray dirn eigen: " << cov_mat.inverse()*ray_dirn_eigen << std::endl;
        std::cout << "t num, t denom: " << t_num << " " << t_denom << std::endl;
	std::cout << "this dist_along_ray, q: " << dist_along_ray << " " << q << std::endl;
    } 

    return std::make_tuple(maha_dist_to_mu, dist_along_ray);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
EllipsoidModelSim::simPtsGivenIntersections(const std::vector<std::vector<int> > &intersection_flag, const std::vector<std::vector<double> > &dist_along_ray)
{
    size_t n_rays = intersection_flag.size();

    Pts sim_pts(n_rays, std::vector<double>(3, 0));
    std::vector<int> hit_flag(n_rays, 0);

    for(size_t i = 0; i < n_rays; ++i)
    {
	if (!anyNonzeros(intersection_flag[i]))
	{
	    hit_flag[i] = 0;
	    continue;
	}
	else
	    hit_flag[i] = 1;

	std::vector<int> sorted_intersecting_ids;
	std::vector<double> sorted_dist_along_ray_intersections;
	std::tie(sorted_intersecting_ids, sorted_dist_along_ray_intersections) =
	    sortIntersectionFlag(intersection_flag[i], dist_along_ray[i]);

	std::vector<double> hit_prob_vec(sorted_intersecting_ids.size());
	for(size_t j = 0; j < sorted_intersecting_ids.size(); ++j)
	    hit_prob_vec[j] = m_ellipsoid_models[sorted_intersecting_ids[j]].hit_prob;

	int hit_ellipsoid_id;
	bool hit_bool;
	std::tie(hit_ellipsoid_id, hit_bool) =
	    sampleHitId(hit_prob_vec, sorted_intersecting_ids, m_deterministic_sim);

	if (!hit_bool)
	{
	    hit_flag[i] = 0;
	    continue;
	}
	 
	std::vector<double> sim_pt = sampleFromMvn(m_ellipsoid_models[hit_ellipsoid_id].mu,
						   m_ellipsoid_models[hit_ellipsoid_id].cov_mat,
						   m_deterministic_sim);
	
	for(size_t j = 0; j < 3; ++j)
	    sim_pts[i][j] = sim_pt[j];
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::vector<double> EllipsoidModelSim::calcMahaDistPtToEllipsoids(const std::vector<int> &ellipsoid_ids, const std::vector<double> &pt)
{
    std::vector<double> maha_dists_to_ellipsoids;
    for(size_t i = 0; i < ellipsoid_ids.size(); ++i)
	maha_dists_to_ellipsoids.push_back(
	    calcMahaDistPtToEllipsoid(ellipsoid_ids[i], pt));

    return maha_dists_to_ellipsoids;
}

double EllipsoidModelSim::calcMahaDistPtToEllipsoid(const int ellipsoid_id, const std::vector<double> &pt)
{
    return calcMahaDistPtToEllipsoid(m_ellipsoid_models[ellipsoid_id].mu, m_ellipsoid_models[ellipsoid_id].cov_mat, 
				 pt);
}

double EllipsoidModelSim::calcMahaDistPtToEllipsoid(const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat, 
						    const std::vector<double> &pt)
{
    Eigen::MatrixXd mu_eigen = stlVecToEigen(mu);
    Eigen::MatrixXd pt_eigen = stlVecToEigen(pt);
    double maha_dist = ((pt_eigen-mu_eigen).transpose()*(cov_mat.inverse()*(pt_eigen-mu_eigen)))(0);
    maha_dist = std::sqrt(maha_dist);
    
    return maha_dist;
}

std::tuple<int, std::vector<int> >
EllipsoidModelSim::assignEllipsoidHitCredits(const std::vector<double> &maha_dists_to_ellipsoids, 
					     const std::vector<int> &sorted_intersecting_ids,
					     const std::vector<double> &sorted_dist_along_ray,
					     const double measured_range)
{
    std::vector<int> flag(maha_dists_to_ellipsoids.size(), 0);
    for(size_t i = 0; i < flag.size(); ++i)
	if (maha_dists_to_ellipsoids[i] < m_max_maha_dist_for_hit)
	    flag[i] = 1;

    int ellipsoid_hit_id = -1;
    std::vector<int> ellipsoid_miss_ids;

    if (!anyNonzeros(flag))
    {
	for(size_t i = 0; i < sorted_dist_along_ray.size(); ++i)
	    if (sorted_dist_along_ray[i] < measured_range)
		ellipsoid_miss_ids.push_back(sorted_intersecting_ids[i]);

	return std::make_tuple(-1, ellipsoid_miss_ids);
    }

    std::vector<int> posns = findNonzeroIds(flag);
    int posn = posns[0]; // take first hit
    ellipsoid_hit_id = sorted_intersecting_ids[posn];
    
    for(size_t i = 0; i < (size_t)posn; ++i)
	ellipsoid_miss_ids.push_back(sorted_intersecting_ids[i]);

    return std::make_tuple(ellipsoid_hit_id, ellipsoid_miss_ids);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
EllipsoidModelSim::simPtsGivenPose(const std::vector<double> &imu_pose)
{
    // rays
    std::vector<double> ray_origin = laserPosnFromImuPose(imu_pose, m_laser_calib_params);
    std::vector<std::vector<double> > ray_dirns = genRayDirnsWorldFrame(imu_pose, m_laser_calib_params);

    return simPtsGivenRays(ray_origin, ray_dirns);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> > 
EllipsoidModelSim::simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses)
{
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    for(size_t i = 0; i < imu_poses.size(); ++i)
    {
	std::vector<std::vector<double> > this_sim_pts;
	std::vector<int> this_hit_flag;
	std::tie(this_sim_pts, this_hit_flag) = simPtsGivenPose(imu_poses[i]); 
	for(size_t j = 0; j < this_sim_pts.size(); ++j)
	{
	    sim_pts.push_back(this_sim_pts[j]);
	    hit_flag.push_back(this_hit_flag[j]);
	}
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<std::vector<double> >, std::vector<int> >
EllipsoidModelSim::simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns)
{
    // intersections
    std::vector<std::vector<int> > intersection_flag;
    std::vector<std::vector<double> > dist_along_ray;
    std::tie(intersection_flag, dist_along_ray) = calcEllipsoidIntersections(
    	ray_origin, ray_dirns);

    // sim
    std::vector<std::vector<double> > sim_pts;
    std::vector<int> hit_flag;
    std::tie(sim_pts, hit_flag) = simPtsGivenIntersections(intersection_flag, dist_along_ray);

    applyMaxRangeFilter(ray_origin, sim_pts, hit_flag, 
			m_laser_calib_params.intrinsics.max_range);

    // todo: comment/ delete
    // std::cout << "EllipsoidModelSim: anyNonzeros(intersection_flag): " 
    // 	      << anyNonzeros(intersection_flag[0]) << std::endl;	

    return std::make_tuple(sim_pts, hit_flag);
}

void EllipsoidModelSim::setDeterministicSim(const bool choice)
{
    m_deterministic_sim = choice;
}
