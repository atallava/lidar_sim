#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/VizUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>

using namespace lidar_sim;

EllipsoidModelSim::EllipsoidModelSim() :
    m_maxMahaDistForHit(3.5)
{    
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

std::tuple<std::vector<std::vector<int> >,
	   std::vector<std::vector<double> > > EllipsoidModelSim::calcEllipsoidIntersections(std::vector<double> ray_origin, std::vector<std::vector<double> > ray_dirns)
{
    size_t n_ellipsoids = m_ellipsoid_models.size();
    // todo: remove!
    // n_ellipsoids = 1;

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
	    // std::cout << "mu, cov_mat: " << std::endl;
	    // dispVec(mu); 
	    // std::cout << cov_mat << std::endl;
	    // std::cout << "this maha dist, dist along ray: " << this_maha_dist << " " << this_dist_along_ray << std::endl;
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
    // std::cout << "t num, t denom: " << t_num << " " << t_denom << std::endl;
    // std::cout << "this dist_along_ray, q: " << dist_along_ray << " " << q << std::endl;

    return std::make_tuple(maha_dist_to_mu, dist_along_ray);
}
