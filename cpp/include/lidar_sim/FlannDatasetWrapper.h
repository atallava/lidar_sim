#pragma once
#include <vector>
#include <string>

#include <flann/flann.hpp>

namespace lidar_sim {
    class FlannDatasetWrapper {
    public:
	FlannDatasetWrapper(const std::vector<std::vector<double> > &dataset);
	void setDataset(const std::vector<std::vector<double> > &dataset);
	

    private:
	int m_n_kd_trees;
	int m_n_checks;
	std::vector<std::vector<double> > m_dataset;
	flann::Matrix<double> m_dataset_flann;
	std::string m_rel_path_index;
    };
}
