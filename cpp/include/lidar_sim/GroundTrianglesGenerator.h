#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <lidar_sim/CgalTypedefs.h>

namespace lidar_sim {
    class GroundTrianglesGenerator {
    public:
	GroundTrianglesGenerator();
	void createTriangleModels(const std::string rel_path_imu_posns);
	void createTriangleModels(const std::vector<std::vector<double> > &imu_posns);
	void fitPts(const std::vector<std::vector<double> > &imu_posns);
	void delaunayTriangulate();
	void calcTrianglesFromTriangulation();
	void writeTrianglesToFile(std::string rel_path_output);
	void writeTrianglesFitPtsToFile(std::string rel_path_output);
	void setDebugFlag(int flag);
	void filterTriangles();

	// not 'fit' to anything, but retain naming from triangle modeler
	// triangles are with respect to these fit pts
	std::vector<std::vector<double> > m_fit_pts;
	std::vector<std::vector<int> > m_triangles;
	std::vector<double> m_hit_prob_vec;
	Delaunay_cgal m_triangulation;

    private:
	int m_debug_flag;
	double m_avg_ground_depth_from_imu;
	double m_lateral_dist;
	double m_fit_pts_node_resn;
	double m_max_triangle_side;
	double m_default_hit_prob;
    };
}
