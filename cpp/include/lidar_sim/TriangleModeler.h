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
#include <lidar_sim/CgalTypedefs.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/SectionLoader.h>

namespace lidar_sim {
    class TriangleModeler {
    public:
	TriangleModeler();
	void createTriangleModels(const std::string rel_path_pts);
	void loadPts(const std::string rel_path_pts);
	void fitSmoothedSurface();
	void fitSmoothedPts();
	std::vector<std::vector<double> >
	    cutNodesToPtsProjn(std::vector<std::vector<double> > xy_nodes);
	std::tuple<std::vector<double>, std::vector<double> >
	    genNodeVecsForSmoothedFit(const std::vector<std::vector<double> > &pts);
	void delaunayTriangulate();
	void calcTrianglesFromTriangulation();
	void writeTrianglesToFile(std::string rel_path_output);
	void setDebugFlag(int flag);
	void calcHitProb(std::string rel_path_section, const PoseServer &imu_poses_server);
	void calcHitProb(const SectionLoader &section, const std::vector<int> &section_pt_ids_to_process, const PoseServer &imu_poses_server);
	void subsamplePts();

	std::vector<std::vector<double> > m_pts;
	alglib::rbfmodel m_surface_model;
	std::vector<std::vector<double> > m_fit_pts;
	std::vector<std::vector<int> > m_triangles;
	std::vector<double> m_hit_prob_vec;
	Delaunay_cgal m_triangulation;

    private:
	int m_debug_flag;
	double m_max_pts_for_surface;
	double m_rbf_radius;
	double m_rbf_layers;
	double m_rbf_reg;
	double m_fit_pts_padding;
	double m_fit_pts_node_resn;
	double m_max_dist_to_projn;
	double m_default_hit_prob;
	int m_hit_count_prior;
	int m_miss_count_prior;
    };
}
