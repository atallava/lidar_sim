#include <iostream>

#include <flann/flann.hpp>

#include <lidar_sim/FlannDatasetWrapper.h>

using namespace lidar_sim;

int main(int argc, char**argv)
{
    // dataset
    std::vector<std::vector<double> > dataset {{0, 0}, {3, 0}, {3, 4}, {0, 1.5}};

    std::vector<double> dataset_unrolled;
    for (size_t i = 0; i < dataset.size(); ++i)
	dataset_unrolled.insert(dataset_unrolled.end(), dataset[i].begin(), dataset[i].end());

    // disp
    std::cout << "dataset" << std::endl;
    for(size_t i = 0; i < dataset.size(); ++i)
    {
	for(size_t j = 0; j < dataset[0].size(); ++j)
	    std::cout << dataset[i][j] << " ";
	std::cout << std::endl;
    }

    std::cout << "dataset unrolled. row major order." << std::endl;
    for(size_t i = 0; i < dataset_unrolled.size(); ++i)
	std::cout << dataset_unrolled[i] << " ";
    std::cout << std::endl << std::endl;

    // wrapping dataset
    FlannDatasetWrapper wrapper(dataset);
    
    // passing dataset_unrolled pointer
    flann::Matrix<double> dataset_flann_2
    	(dataset_unrolled.data(), dataset.size(), dataset[0].size());

    // disp
    std::cout << "flann dataset 2. " << std::endl;
    for(size_t i = 0; i < dataset.size(); ++i)
    {
	for(size_t j = 0; j < dataset[0].size(); ++j)
	    std::cout << dataset_flann_2[i][j] << " ";
	std::cout << std::endl;
    }
    std::cout << std::endl;

    // query
    std::vector<std::vector<double> > query {{-0.5, 0.5}, {2, 1}};

    std::vector<double> query_unrolled;
    for (size_t i = 0; i < query.size(); ++i)
	query_unrolled.insert(query_unrolled.end(), query[i].begin(), query[i].end());

    // disp
    std::cout << "query" << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < query[0].size(); ++j)
	    std::cout << query[i][j] << " ";
	std::cout << std::endl;
    }

    std::cout << "query unrolled. row major order." << std::endl;
    for(size_t i = 0; i < query_unrolled.size(); ++i)
	std::cout << query_unrolled[i] << " ";
    std::cout << std::endl << std::endl;
    
    // passing query_unrolled pointer
    flann::Matrix<double> query_flann_2
	(query_unrolled.data(), query.size(), query[0].size());

    // disp
    std::cout << "flann query. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < query[0].size(); ++j)
	    std::cout << query_flann_2[i][j] << " ";
	std::cout << std::endl;
    }
    std::cout << std::endl;

    // indices
    int n_kd_trees = 10;
    // wrapper builds internal index
    flann::Index<flann::L2<double> > index_2(dataset_flann_2, flann::KDTreeIndexParams(n_kd_trees));
    index_2.buildIndex();

    // flann output
    int nn = 2;
    int n_checks = 100;
    std::vector<std::vector<int> > indices_1;
    std::vector<std::vector<double> > dists_1;
    std::tie(indices_1, dists_1) = wrapper.knnSearch(query, nn);

    // disp
    std::cout << "indices 1. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << indices_1[i][j] << " ";
	std::cout << std::endl;
    }
    
    std::cout << "dists 1. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << dists_1[i][j] << " ";
	std::cout << std::endl;
    }
    std::cout << std::endl;

    // stl output
    std::vector<std::vector<int> > indices_2;
    std::vector<std::vector<double> > dists_2;
    index_2.knnSearch(query_flann_2, indices_2, dists_2, nn, flann::SearchParams(n_checks));

    // disp
    std::cout << "indices 2. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << indices_2[i][j] << " ";
	std::cout << std::endl;
    }
    
    std::cout << "dists 2. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << dists_2[i][j] << " ";
	std::cout << std::endl;
    }
    std::cout << std::endl;

    return(1);
}
