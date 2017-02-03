#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "interpolation.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/LaserCalibParams.h>

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

    private:
	std::vector<std::vector<double> > m_pts;
	EllipsoidModels m_ellipsoid_models;
	int m_debug_flag;
	alglib::integer_1d_array m_pt_cluster_ids;
	std::vector<int> m_selected_cluster_ids;
	double m_n_clusters_per_pt;
	int m_min_pts_per_cluster;
	int m_n_clusters;
    };
}
