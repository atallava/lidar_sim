#include <algorithm>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>

using namespace lidar_sim;

EllipsoidModelSim::EllipsoidModelSim() :
    m_maxMahaDistForHit(3.5),
    m_debug_flag(0)
{    
    std::random_device rd;
    std::mt19937 gen(rd());
    m_gen = gen;
}

void EllipsoidModelSim::setEllipsoidModels(EllipsoidModels ellipsoid_models)
{
    m_ellipsoid_models = ellipsoid_models;
}

void EllipsoidModelSim::setEllipsoidModels(std::string rel_path_models)
{
    m_ellipsoid_models = loadEllipsoidModels(rel_path_models);
}

void EllipsoidModelSim::setLaserCalibParams(LaserCalibParams laser_calib_params)
{
    m_laser_calib_params = laser_calib_params;
}

void EllipsoidModelSim::setDebugFlag(int value)
{
    m_debug_flag = value;
}

std::tuple<std::vector<std::vector<int> >,
	   std::vector<std::vector<double> > > EllipsoidModelSim::calcEllipsoidIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns)
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
	    if ((maha_dist_to_mu[i][j] <= m_maxMahaDistForHit) && 
		(dist_along_ray[i][j] <= m_laser_calib_params.intrinsics.max_range) &&
		(dist_along_ray[i][j] >= m_laser_calib_params.intrinsics.min_range))
		intersection_flag[i][j] = 1;
	    else
		intersection_flag[i][j] = 0;
    
    return std::make_tuple(intersection_flag, dist_along_ray);
}

std::tuple<double, double> EllipsoidModelSim::calcMahaDistRayToEllipsoid(std::vector<double> ray_origin, std::vector<double> ray_dirn, std::vector<double> mu, Eigen::MatrixXd cov_mat)
{
    // TODO: bad conversion work
    
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
EllipsoidModelSim::simPtsGivenIntersections(std::vector<std::vector<int> > intersection_flag, std::vector<std::vector<double> > dist_along_ray)
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
	    sampleHitId(hit_prob_vec, sorted_intersecting_ids);

	if (!hit_bool)
	{
	    hit_flag[i] = 0;
	    continue;
	}
	 
	std::vector<double> sim_pt = sampleFromMvn(m_ellipsoid_models[hit_ellipsoid_id].mu,
						   m_ellipsoid_models[hit_ellipsoid_id].cov_mat);
	for(size_t j = 0; j < 3; ++j)
	    sim_pts[i][j] = sim_pt[j];
    }

    return std::make_tuple(sim_pts, hit_flag);
}

std::tuple<std::vector<int>, std::vector<double> >
EllipsoidModelSim::sortIntersectionFlag(std::vector<int> intersection_flag, std::vector<double> dist_along_ray)
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

std::tuple<int, bool>
EllipsoidModelSim::sampleHitId(std::vector<double> hit_prob_vec, std::vector<int> target_ids)
{
    size_t n_targets = hit_prob_vec.size();
    std::uniform_real_distribution<> dis(0, 1);

    int hit_id;
    bool hit_bool;
    for(size_t i = 0; i < n_targets; ++i)
	if (dis(m_gen) < hit_prob_vec[i])
	{
	    hit_id = target_ids[i];
	    hit_bool = true;
	    return std::make_tuple(hit_id, hit_bool);
	}
    hit_id = -1;
    hit_bool = false;
    return std::make_tuple(hit_id, hit_bool);
}

