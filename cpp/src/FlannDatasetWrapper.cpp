#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <stdexcept>
#include <math.h>
#include <chrono>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/FlannDatasetWrapper.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

FlannDatasetWrapper::FlannDatasetWrapper() :
    m_n_kd_trees(10), 
    m_n_checks(500)
{
    uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>
	(std::chrono::steady_clock::now().time_since_epoch()).count();

    std::ostringstream ss;
    ss << "flann_index_" << t;
    m_rel_path_index = ss.str();
}

FlannDatasetWrapper::FlannDatasetWrapper(const std::vector<std::vector<double> > &dataset) :
    m_n_kd_trees(10),
    m_n_checks(500)
{
    uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>
	(std::chrono::steady_clock::now().time_since_epoch()).count();

    std::ostringstream ss;
    ss << "flann_index_" << t;
    m_rel_path_index = ss.str();

    setDataset(dataset);
}

FlannDatasetWrapper::~FlannDatasetWrapper()
{
    std::remove(m_rel_path_index.c_str());
}

void FlannDatasetWrapper::setDataset(const std::vector<std::vector<double> > &dataset)
{
    m_dataset = dataset;
    m_dataset_flann = stlArrayToFlannMatrix(m_dataset);
    flann::Index<flann::L2<double> > index(m_dataset_flann, flann::KDTreeIndexParams(m_n_kd_trees));
    index.buildIndex();
    index.save(m_rel_path_index);
}

std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
FlannDatasetWrapper::knnSearch(const std::vector<std::vector<double> > &pts, const int nn)
{
    flann::Matrix<double> queries = stlArrayToFlannMatrix(pts);
    flann::Matrix<int> indices(new int[queries.rows*nn], queries.rows, nn);
    flann::Matrix<double> dists(new double[queries.rows*nn], queries.rows, nn);
    
    // load index
    flann::Index<flann::L2<double> > index(m_dataset_flann, flann::SavedIndexParams(m_rel_path_index));
    index.knnSearch(queries, indices, dists, nn, flann::SearchParams(m_n_checks));

    std::vector<std::vector<double> > nn_dists = flannMatrixToStlArray(dists);
    // flann returns squared distances
    for(size_t i = 0; i < nn_dists.size(); ++i)
	for(size_t j = 0 ; j < nn_dists[i].size(); ++j)
	    nn_dists[i][j] = std::sqrt(nn_dists[i][j]);
	
    return std::make_tuple(
	flannMatrixToStlArray(indices), nn_dists);
}

std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
FlannDatasetWrapper::knnSearch(const std::vector<double> &pt, const int nn)
{
    std::vector<std::vector<double> > pts = wrapDataInVec(pt);
    return knnSearch(pts, nn);
}

std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
FlannDatasetWrapper::radiusSearch(const std::vector<std::vector<double> > &pts, const double radius)
{
    flann::Matrix<double> queries = stlArrayToFlannMatrix(pts);
    std::vector<std::vector<int> > indices;
    std::vector<std::vector<double> > dists;
    
    // load index
    flann::Index<flann::L2<double> > index(m_dataset_flann, flann::SavedIndexParams(m_rel_path_index));
    // note: since using L2 distance, raising radius to pow 2 
    index.radiusSearch(queries, indices, dists, std::pow(radius, 2.0), flann::SearchParams(m_n_checks));

    // flann returns squared distances
    for(size_t i = 0; i < dists.size(); ++i)
    	for(size_t j = 0 ; j < dists[i].size(); ++j)
    	    dists[i][j] = std::sqrt(dists[i][j]);
	
    return std::make_tuple(indices, dists);
}

std::tuple<int, double> FlannDatasetWrapper::approxNearestToRay(const std::vector<double> &ray_origin, 
								const std::vector<double> &ray_dirn, const double max_ray_length, const double resn_along_ray)
{
    std::vector<std::vector<double> > pts_along_ray = getPtsAlongRay(ray_origin, ray_dirn, 
								     max_ray_length, resn_along_ray);

    std::vector<std::vector<int> > wrapped_indices;
    std::vector<std::vector<double> > wrapped_dists;
    std::tie(wrapped_indices, wrapped_dists) = knnSearch(pts_along_ray, 1);
    
    std::vector<int> indices;
    std::vector<double> dists;
    for (size_t i = 0; i < pts_along_ray.size(); ++i) {
	indices.push_back(wrapped_indices[i][0]);
	dists.push_back(wrapped_dists[i][0]);
    }
    
    auto min_it = std::min_element(dists.begin(), dists.end());
    int min_idx = std::distance(dists.begin(), min_it);
    double min_dist = dists[min_idx];

    return std::tie(min_idx, min_dist);
}

std::tuple<std::vector<int>, std::vector<double> > FlannDatasetWrapper::approxNearestToRays(const std::vector<double> &ray_origin, 
											   const std::vector<std::vector<double> > &ray_dirns, const double max_ray_length, const double resn_along_ray)
{
    // get pts along ray for each ray, concatenate into one big one
    std::vector<std::vector<double> > nodes_all;
    std::vector<int> n_nodes_per_ray;
    size_t n_rays = ray_dirns.size();
    for (size_t i = 0; i < n_rays; ++i)
    {
	std::vector<std::vector<double> > this_ray_nodes = getPtsAlongRay(ray_origin, ray_dirns[i], max_ray_length, resn_along_ray);
	nodes_all.insert(nodes_all.end(), this_ray_nodes.begin(), this_ray_nodes.end());
	n_nodes_per_ray.push_back(this_ray_nodes.size());
    }

    std::vector<std::vector<int> > wrapped_indices_nodes_all;
    std::vector<std::vector<double> > wrapped_dists_nodes_all;
    std::tie(wrapped_indices_nodes_all, wrapped_dists_nodes_all) = knnSearch(nodes_all, 1);

    std::vector<int> indices_nodes_all;
    std::vector<double> dists_nodes_all;
    for (size_t i = 0; i < nodes_all.size(); ++i) {
	indices_nodes_all.push_back(wrapped_indices_nodes_all[i][0]);
	dists_nodes_all.push_back(wrapped_dists_nodes_all[i][0]);
    }

    std::vector<int> indices(n_rays, 0);
    std::vector<double> dists(n_rays, 0.0);

    size_t count = 0;
    for (size_t i = 0; i < n_rays; ++i)
    {
	// todo: this can be faster
	size_t this_ray_n_nodes = n_nodes_per_ray[i];
	std::vector<double> this_ray_dists;
	for (size_t j = count; j < count + this_ray_n_nodes; ++j)
	    this_ray_dists.push_back(dists_nodes_all[j]);

	count = count + this_ray_n_nodes;

	auto min_it = std::min_element(this_ray_dists.begin(), this_ray_dists.end());
	int min_idx = std::distance(this_ray_dists.begin(), min_it);
	double min_dist = this_ray_dists[min_idx];

	indices[i] = min_idx;
	dists[i] = min_dist;
    }

    return std::tie(indices, dists);
}
