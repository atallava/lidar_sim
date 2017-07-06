#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <math.h>
#include <chrono>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/EllipsoidSimNbrServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

EllipsoidSimNbrServer::EllipsoidSimNbrServer() :
    m_ellipsoid_nn_radius(5)
{
}

EllipsoidSimNbrServer::EllipsoidSimNbrServer(const std::vector<EllipsoidModel> &ellipsoid_models) :
    m_ellipsoid_nn_radius(5) 
{
    setEllipsoidModels(ellipsoid_models);
}

void EllipsoidSimNbrServer::setEllipsoidModels(const std::vector<EllipsoidModel> &ellipsoid_models)
{
    m_ellipsoid_models = ellipsoid_models;
    m_n_ellipsoids = m_ellipsoid_models.size();
    std::vector<std::vector<double> > ellipsoid_centers(m_n_ellipsoids,
							std::vector<double>(3, 0));
    for (size_t i = 0; i < (size_t)m_n_ellipsoids; ++i)
	ellipsoid_centers[i] = m_ellipsoid_models[i].mu;
    m_ellipsoid_centers = ellipsoid_centers;

    m_flann_helper.setDataset(m_ellipsoid_centers);
}

EllipsoidModelSim EllipsoidSimNbrServer::createSim(const std::vector<double> &pt)
{
    return createSim(wrapDataInVec(pt));
}

EllipsoidModelSim EllipsoidSimNbrServer::createSim(const std::vector<std::vector<double> > &pts)
{
    std::vector<std::vector<int> > ids;
    std::vector<std::vector<double> > dists;
    std::tie(ids,dists) = m_flann_helper.radiusSearch(pts,
						      m_ellipsoid_nn_radius);
    std::vector<int> unique_ids;
    for(size_t i = 0; i < ids.size(); ++i)
	unique_ids.insert(unique_ids.end(), 
			  ids[i].begin(), ids[i].end());
    unique_ids = getUniqueSortedVec(unique_ids);

    // debug
    // std::cout << "unique ids: " << std::endl;
    // dispVec(unique_ids);

    std::vector<EllipsoidModel> ellipsoids_nbr;
    for (size_t i = 0; i < unique_ids.size(); ++i)
    {
	int id = unique_ids[i];
	ellipsoids_nbr.push_back(m_ellipsoid_models[id]);
    }

    return createSimGivenEllipsoids(ellipsoids_nbr);
}

EllipsoidModelSim EllipsoidSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<double> &ray_dirn)
{
    return createSim(getNodesAlongRay(ray_origin, ray_dirn));
}

EllipsoidModelSim EllipsoidSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<std::vector<double> > &ray_dirns)
{
    return createSim(getNodesAlongRays(ray_origin, ray_dirns));
}

EllipsoidModelSim EllipsoidSimNbrServer::createSimGivenEllipsoids(const std::vector<EllipsoidModel> &ellipsoid_models)
{
    EllipsoidModelSim sim;
    // note: determinsitc sim and laser calib params set outside
    sim.setEllipsoidModels(ellipsoid_models); 

    return sim;
}

std::vector<std::vector<double> > EllipsoidSimNbrServer::getNodesAlongRay(const std::vector<double> &ray_origin, 
									  const std::vector<double> &ray_dirn)
{
    // dists along ray
    double max_range = m_laser_calib_params.intrinsics.max_range;
    std::vector<double> dists_along_ray;
    double walker = m_laser_calib_params.intrinsics.min_range;
    while (walker < max_range)
    {
	dists_along_ray.push_back(walker);
	walker += m_ellipsoid_nn_radius;
    }
    dists_along_ray.push_back(max_range);
    
    // nodes along ray
    std::vector<std::vector<double> > nodes_along_ray;
    size_t n_nodes = dists_along_ray.size();
    for(size_t i = 0; i < n_nodes; ++i)
    {
	double dist_along_ray = dists_along_ray[i];
	std::vector<double> node(3,0);
	for(size_t j = 0; j < 3; ++j)
	    node[j] = ray_origin[j] + dist_along_ray*ray_dirn[j];

	nodes_along_ray.push_back(node);
    }

    // debug
    // std::cout << "nodes along ray: " << std::endl;
    // dispMat(nodes_along_ray);
    
    return nodes_along_ray;
}

std::vector<std::vector<double> > EllipsoidSimNbrServer::getNodesAlongRays(const std::vector<double> &ray_origin, 
									   const std::vector<std::vector<double> > &ray_dirns)
{
    std::vector<std::vector<double> > nodes_along_rays;
    for (size_t i = 0; i < ray_dirns.size(); ++i)
    {
	std::vector<double> ray_dirn = ray_dirns[i];
	std::vector<std::vector<double> > nodes_along_ray = 
	    getNodesAlongRay(ray_origin, ray_dirn);
	nodes_along_rays.insert(nodes_along_rays.end(),
				nodes_along_ray.begin(), nodes_along_ray.end());
    }
    
    return nodes_along_rays;
}
