#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <math.h>
#include <chrono>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/TriangleSimNbrServer.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

TriangleSimNbrServer::TriangleSimNbrServer() :
    m_nn_radius(5)
{
}

TriangleSimNbrServer::TriangleSimNbrServer(const std::vector<TriangleModels> &triangle_models_vec) :
    m_nn_radius(5) 
{
    setTriangleModels(triangle_models_vec);
}

void TriangleSimNbrServer::setTriangleModels(const std::vector<TriangleModels> &triangle_models_vec)
{
    m_triangle_models_vec = triangle_models_vec;
    m_triangle_models = stitchTriangleModels(m_triangle_models_vec);

    m_flann_helper.setDataset(m_triangle_models.m_fit_pts);
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &pt)
{
    return createSim(wrapDataInVec(pt));
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<std::vector<double> > &pts)
{
    std::vector<std::vector<int> > ids;
    std::vector<std::vector<double> > dists;
    std::tie(ids,dists) = m_flann_helper.radiusSearch(pts,
						      m_nn_radius);
    std::vector<int> unique_ids;
    for(size_t i = 0; i < ids.size(); ++i)
	unique_ids.insert(unique_ids.end(), 
			  ids[i].begin(), ids[i].end());
    unique_ids = getUniqueSortedVec(unique_ids);

    // get triangle ids involved in the pt ids
    std::vector<int> flag;
    std::vector<std::vector<int> > triangles_nbr;
    std::vector<double> hit_prob_vec_nbr;
    for (size_t i = 0; i < m_triangle_models.m_triangles.size(); ++i)
    {
	std::vector<int> vertex_ids = m_triangle_models.m_triangles[i];
	double hit_prob = m_triangle_models.m_hit_prob_vec[i];
	for (size_t j = 0; j < 3; ++j)
	{
	    bool condn = std::find(unique_ids.begin(), unique_ids.end(), vertex_ids[j]) 
		!= unique_ids.end();
	    if (condn)
	    {
		// add to triangles nbr
		flag[i] = 1;
		triangles_nbr.push_back(vertex_ids);
		hit_prob_vec_nbr.push_back(hit_prob);
		break;
	    }
	}
    }

    TriangleModels triangle_models_nbr;
    // todo: remove baggage of extra fit pts
    triangle_models_nbr.m_fit_pts = m_triangle_models.m_fit_pts;
    triangle_models_nbr.m_triangles = triangles_nbr;
    triangle_modrls_nbr.m_hit_prob_vec = hit_prob_vec_nbr;

    // debug
    // std::cout << "unique ids: " << std::endl;
    // dispVec(unique_ids);

    return createSimGivenTriangleModels(triangle_models_nbr);
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<double> &ray_dirn)
{
    return createSim(getNodesAlongRay(ray_origin, ray_dirn));
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<std::vector<double> > &ray_dirns)
{
    return createSim(getNodesAlongRays(ray_origin, ray_dirns));
}

TriangleModelSim TriangleSimNbrServer::createSimGivenTriangleModels(const TriangleModels &triangle_models)
{
    TriangleModelSim sim;
    // note: determinsitc sim and laser calib params set outside
    sim.setTriangleModels(triangle_models); 

    return sim;
}

std::vector<std::vector<double> > TriangleSimNbrServer::getNodesAlongRay(const std::vector<double> &ray_origin, 
									  const std::vector<double> &ray_dirn)
{
    // dists along ray
    double max_range = m_laser_calib_params.intrinsics.max_range;
    std::vector<double> dists_along_ray;
    double walker = m_laser_calib_params.intrinsics.min_range;
    while (walker < max_range)
    {
	dists_along_ray.push_back(walker);
	walker += m_nn_radius;
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

std::vector<std::vector<double> > TriangleSimNbrServer::getNodesAlongRays(const std::vector<double> &ray_origin, 
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
