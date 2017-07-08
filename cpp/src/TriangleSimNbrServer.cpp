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
    m_nn_radius(5) // todo: 5
{
}

TriangleSimNbrServer::TriangleSimNbrServer(const std::vector<TriangleModels> &triangle_models_vec) :
    m_nn_radius(1) // todo: 5
{
    setTriangleModels(triangle_models_vec);
}

void TriangleSimNbrServer::setTriangleModels(const std::vector<TriangleModels> &triangle_models_vec)
{
    m_triangle_models_vec = triangle_models_vec;
    m_triangle_models = stitchTriangleModels(m_triangle_models_vec);
    m_n_triangles = m_triangle_models.m_triangles.size();
    std::vector<std::vector<double> > triangle_centers(m_n_triangles,
						       std::vector<double>(3, 0));
    for (size_t i = 0; i < (size_t)m_n_triangles; ++i)
    {
	std::vector<std::vector<double> > vertices;
	for (size_t j = 0; j < 3; ++j)
	    vertices.push_back(
		m_triangle_models.m_fit_pts[
		    m_triangle_models.m_triangles[i][j]]);

	triangle_centers[i] = calcPtsMean(vertices);
    }
    m_triangle_centers = triangle_centers;

    // todo: decide course
    // m_flann_helper.setDataset(m_triangle_models.m_fit_pts);
    m_flann_helper.setDataset(m_triangle_centers);
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &pt)
{
    return createSim(wrapDataInVec(pt));
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<std::vector<double> > &pts)
{
    // todo: remove this timing
    const clock_t begin_time = std::clock();

    std::vector<std::vector<int> > ids;
    std::vector<std::vector<double> > dists;
    std::tie(ids,dists) = m_flann_helper.radiusSearch(pts,
						      m_nn_radius);
    std::vector<int> unique_ids;
    for(size_t i = 0; i < ids.size(); ++i)
	unique_ids.insert(unique_ids.end(), 
			  ids[i].begin(), ids[i].end());
    unique_ids = getUniqueSortedVec(unique_ids);

    // todo: remove this timing
    std::cout << "radius search time: " << float(std::clock() - begin_time)/ CLOCKS_PER_SEC << std::endl; 
    std::cout << "dataset size: " << m_triangle_centers.size() << std::endl;

    // todo: decide course
    // get triangle ids involved in the pt ids
    // std::vector<int> flag(m_triangle_models.m_triangles.size(), 0);
    // std::vector<std::vector<int> > triangles_nbr;
    // std::vector<double> hit_prob_vec_nbr;
    // for (size_t i = 0; i < m_triangle_models.m_triangles.size(); ++i)
    // {
    // 	std::vector<int> vertex_ids = m_triangle_models.m_triangles[i];
    // 	double hit_prob = m_triangle_models.m_hit_prob_vec[i];
    // 	for (size_t j = 0; j < 3; ++j)
    // 	{
    // 	    bool condn = std::find(unique_ids.begin(), unique_ids.end(), vertex_ids[j]) 
    // 		!= unique_ids.end();
    // 	    if (condn)
    // 	    {
    // 		// add to triangles nbr
    // 		flag[i] = 1;
    // 		triangles_nbr.push_back(vertex_ids);
    // 		hit_prob_vec_nbr.push_back(hit_prob);
    // 		break;
    // 	    }
    // 	}
    // }

    // directly add triangles
    std::vector<std::vector<int> > triangles_nbr;
    std::vector<double> hit_prob_vec_nbr;
    for (size_t i = 0; i < unique_ids.size(); ++i)
    {
    	int id = unique_ids[i];
    	triangles_nbr.push_back(m_triangle_models.m_triangles[id]);
    	hit_prob_vec_nbr.push_back(m_triangle_models.m_hit_prob_vec[id]);
    }

    TriangleModels triangle_models_nbr;
    // todo: remove baggage of extra fit pts
    triangle_models_nbr.m_fit_pts = m_triangle_models.m_fit_pts;
    triangle_models_nbr.m_triangles = triangles_nbr;
    triangle_models_nbr.m_hit_prob_vec = hit_prob_vec_nbr;

    // todo: remove this timing
    std::cout << "+ triangles nbr creation time: " << float(std::clock() - begin_time)/ CLOCKS_PER_SEC << std::endl; 

    // debug
    // std::cout << "unique ids: " << std::endl;
    // dispVec(unique_ids);

    return createSimGivenTriangleModels(triangle_models_nbr);
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<double> &ray_dirn)
{
    return createSim(getNodesAlongRay(ray_origin, ray_dirn, m_laser_calib_params, m_nn_radius));
}

TriangleModelSim TriangleSimNbrServer::createSim(const std::vector<double> &ray_origin, 
						   const std::vector<std::vector<double> > &ray_dirns)
{
    return createSim(getNodesAlongRays(ray_origin, ray_dirns, m_laser_calib_params, m_nn_radius));
}

TriangleModelSim TriangleSimNbrServer::createSimGivenTriangleModels(const TriangleModels &triangle_models)
{
    TriangleModelSim sim;
    // note: determinsitc sim and laser calib params set outside
    sim.setTriangleModels(triangle_models); 

    return sim;
}

