#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>
#include <math.h>

#include <boost/algorithm/string.hpp>

#include <lidar_sim/FlannDatasetWrapper.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

FlannDatasetWrapper::FlannDatasetWrapper(const std::vector<std::vector<double> > &dataset) :
    m_n_kd_trees(10),
    m_n_checks(100),
    m_rel_path_index("flann_index")
{
    setDataset(dataset);
}

void FlannDatasetWrapper::setDataset(const std::vector<std::vector<double> > &dataset)
{
    m_dataset = dataset;
    m_dataset_flann = stlArrayToFlannMatrix(m_dataset);
    flann::Index<flann::L2<double> > index(m_dataset_flann, flann::KDTreeIndexParams(m_n_kd_trees));
    index.buildIndex();
    index.save(m_rel_path_index);
}

// std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
// FlannDatasetWrapper::knnsearch(const std::vector<std::vector<double> > &pts, const int nn)
// {
//     flann::Matrix<double> query = stlArrayToFlannMatrix(pts);
//     flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
//     flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);
    
//     // load index

// }

