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
    m_n_checks(100)
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
