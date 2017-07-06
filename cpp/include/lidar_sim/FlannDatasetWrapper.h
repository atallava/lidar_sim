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
	    radiusSearch(const std::vector<std::vector<double> > &pts, const double radius);

    private:
	int m_n_kd_trees;
	int m_n_checks;
	std::vector<std::vector<double> > m_dataset;
	flann::Matrix<double> m_dataset_flann;
	std::string m_rel_path_index;
    };
}
