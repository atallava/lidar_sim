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
#include <lidar_sim/ModelingUtils.h>
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
	void writeEllipsoidsToFile(std::string rel_path_output);
	void calcHitProb(std::string rel_path_section, const PoseServer &imu_poses_server);
	void calcHitProb(const SectionLoader &section, const std::vector<int> &section_pt_ids_to_process, const PoseServer &imu_poses_server);
	void filterPts();
	void setVerbosity(int verbose);

	// hack for patching calcHitProb
	void setEllipsoidModels(const EllipsoidModels &ellipsoid_models);

	std::vector<std::vector<double> > m_pts;
	EllipsoidModels m_ellipsoid_models;
	double m_n_clusters_per_pt;
	bool m_set_max_maha_dist_for_hit; // for sim optim
	double m_max_maha_dist_for_hit; // for sim optim

    private:
	int m_verbose;
	alglib::integer_1d_array m_pt_cluster_ids;
	std::vector<int> m_selected_cluster_ids;
	int m_min_pts_per_cluster;
	int m_n_clusters;

	double m_default_hit_prob;
	int m_hit_count_prior;
	int m_miss_count_prior;
	double m_max_pts_dist_to_nbrs; // used to filter out pts before modeling
    };
}
