#include <iostream>

#include <flann/flann.hpp>

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
    
    // using new
    flann::Matrix<double> dataset_flann_1 
	(new double[dataset.size()*dataset[0].size()], dataset.size(), dataset[0].size());
    for(size_t i = 0; i < dataset.size(); ++i)
	for(size_t j = 0; j < dataset[0].size(); ++j)
	    dataset_flann_1[i][j] = dataset[i][j];

    // passing dataset_unrolled pointer
    flann::Matrix<double> dataset_flann_2
	(dataset_unrolled.data(), dataset.size(), dataset[0].size());

    // disp
    std::cout << "flann dataset 1. " << std::endl;
    for(size_t i = 0; i < dataset.size(); ++i)
    {
	for(size_t j = 0; j < dataset[0].size(); ++j)
	    std::cout << dataset_flann_1[i][j] << " ";
	std::cout << std::endl;
    }
    
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
    
    // using new
    flann::Matrix<double> query_flann_1 
	(new double[query.size()*query[0].size()], query.size(), query[0].size());
    for(size_t i = 0; i < query.size(); ++i)
	for(size_t j = 0; j < query[0].size(); ++j)
	    query_flann_1[i][j] = query[i][j];

    // passing query_unrolled pointer
    flann::Matrix<double> query_flann_2
	(query_unrolled.data(), query.size(), query[0].size());

    // disp
    std::cout << "flann query 1. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < query[0].size(); ++j)
	    std::cout << query_flann_1[i][j] << " ";
	std::cout << std::endl;
    }
    
    std::cout << "flann query 2. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < query[0].size(); ++j)
	    std::cout << query_flann_2[i][j] << " ";
	std::cout << std::endl;
    }
    std::cout << std::endl;

    // indices
    int n_kd_trees = 10;
    flann::Index<flann::L2<double> > index_1(dataset_flann_1, flann::KDTreeIndexParams(n_kd_trees));
    index_1.buildIndex();
    flann::Index<flann::L2<double> > index_2(dataset_flann_2, flann::KDTreeIndexParams(n_kd_trees));
    index_2.buildIndex();

    // flann output
    int nn = 2;
    int n_checks = 100;
    flann::Matrix<int> indices_flann_1(new int[query_flann_1.rows*nn], query_flann_1.rows, nn);
    flann::Matrix<double> dists_flann_1(new double[query_flann_1.rows*nn], query_flann_1.rows, nn);
    index_1.knnSearch(query_flann_1, indices_flann_1, dists_flann_1, nn, flann::SearchParams(n_checks));

    // disp
    std::cout << "flann indices 1. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << indices_flann_1[i][j] << " ";
	std::cout << std::endl;
    }
    
    std::cout << "flann dists 1. " << std::endl;
    for(size_t i = 0; i < query.size(); ++i)
    {
	for(size_t j = 0; j < (size_t)nn; ++j)
	    std::cout << dists_flann_1[i][j] << " ";
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

    // hierarchical clustering
    int n_clusters_queried = 2;
    size_t dim_dataset = dataset[0].size();
    std::vector<std::vector<double> > centers;
    centers.resize(n_clusters_queried, std::vector<double> (dim_dataset));
    std::vector<double> centers_unrolled;
    for (size_t i = 0; i < centers.size(); ++i)
	centers_unrolled.insert(centers_unrolled.end(), centers[i].begin(), centers[i].end());
    
    flann::Matrix<double> centers_flann(centers_unrolled.data(), n_clusters_queried, dim_dataset);

    flann::KMeansIndexParams kmeans_index_params;
    int n_clusters_returned = 
	flann::hierarchicalClustering<flann::L2<double> >(dataset_flann_2, centers_flann, kmeans_index_params);

    // disp
    std::cout << "centers_flann. " << std::endl;
    for(size_t i = 0; i < (size_t)n_clusters_returned; ++i)
    {
	for(size_t j = 0; j < dim_dataset; ++j)
	    std::cout << centers_flann[i][j] << " ";
	std::cout << std::endl;
    }

    std::cout << "centers. " << std::endl;
    for(size_t i = 0; i < centers.size(); ++i)
    {
	for(size_t j = 0; j < centers[i].size(); ++j)
	    std::cout << centers[i][j] << " ";
	std::cout << std::endl;
    }
        
    std::cout << "centers unrolled. " << std::endl;
    for(size_t i = 0; i < centers_unrolled.size(); ++i)
	std::cout << centers_unrolled[i] << " ";
    std::cout << std::endl;
        
    return(1);
}
