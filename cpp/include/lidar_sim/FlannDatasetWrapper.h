#pragma once
#include <vector>
#include <string>

#include <flann/flann.hpp>

namespace lidar_sim {
    class FlannDatasetWrapper {
    public:
	FlannDatasetWrapper();
	FlannDatasetWrapper(const std::vector<std::vector<double> > &dataset);
	~FlannDatasetWrapper();

	void setDataset(const std::vector<std::vector<double> > &dataset);
	std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
	    knnSearch(const std::vector<std::vector<double> > &pts, const int nn = 1);
	std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
	    knnSearch(const std::vector<double> &pt, const int nn = 1);
	std::tuple<std::vector<std::vector<int> >, std::vector<std::vector<double> > >
	    radiusSearch(const std::vector<std::vector<double> > &pts, const double radius);
	std::tuple<int, double> approxNearestToRay(const std::vector<double> &ray_origin, 
						   const std::vector<double> &ray_dirn, const double max_ray_length, const double resn_along_ray);
	std::tuple<std::vector<int>, std::vector<double> > approxNearestToRays(const std::vector<double> &ray_origin, 
									       const std::vector<std::vector<double> > &ray_dirns, const double max_ray_length, const double resn_along_ray);

    private:
	int m_n_kd_trees;
	int m_n_checks;
	std::vector<std::vector<double> > m_dataset;
	std::vector<double> m_dataset_unrolled;
	flann::Matrix<double> m_dataset_flann;
	std::string m_rel_path_index;
    };
}
