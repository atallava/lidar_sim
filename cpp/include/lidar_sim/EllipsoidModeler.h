#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "interpolation.h"

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/SectionLoader.h>

namespace lidar_sim {
    class EllipsoidModeler {
    public:
	EllipsoidModeler();
	void createEllipsoidModels(const std::string rel_path_pts);
	// this is probably bad practice. who knows which variables are needed for which function?
	void loadPts(const std::string rel_path_pts);
	void clusterPts();
	void filterClusters();
	void fillEllipsoidModels();
	int calcNClusters();
	EllipsoidModel createEllipsoidModel(const Pts &pts);
	void writeEllipsoidModelsToFile(std::string rel_path_output);
	void calcHitProb(std::string rel_path_section, PoseServer imu_poses_server);

	std::vector<std::vector<double> > m_pts;
	EllipsoidModels m_ellipsoid_models;
	void setDebugFlag(int flag);

    private:
	int m_debug_flag;
	alglib::integer_1d_array m_pt_cluster_ids;
	std::vector<int> m_selected_cluster_ids;
	double m_n_clusters_per_pt;
	int m_min_pts_per_cluster;
	int m_n_clusters;
	double m_default_hit_prob;

	int m_hit_count_prior;
	int m_miss_count_prior;
    };
}
