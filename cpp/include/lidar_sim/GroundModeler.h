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
    typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel_cgal;
    typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel_cgal> Vb_cgal;
    typedef CGAL::Triangulation_data_structure_2<Vb_cgal>                       Tds_cgal;
    typedef Kernel_cgal::Point_2                                                Point_cgal;
    typedef CGAL::Delaunay_triangulation_2<Kernel_cgal, Tds_cgal> Delaunay_cgal;

    class GroundModeler {
    public:
	GroundModeler();
	void fitSmoothedSurface();
	void fitSmoothedPts();
	std::vector<std::vector<double> >
	    cutNodesToPtsProjn(std::vector<std::vector<double> > xy_nodes);
	std::tuple<std::vector<double>, std::vector<double> >
	    genNodeVecsForSmoothedFit(const std::vector<std::vector<double> > &pts);
	void delaunayTriangulate();

	std::vector<std::vector<double> > m_pts;
	alglib::rbfmodel m_surface_model;
	std::vector<std::vector<double> > m_fit_pts;

    private:
	double m_rbf_radius;
	double m_rbf_layers;
	double m_rbf_reg;
	double m_fit_pts_padding;
	double m_fit_pts_node_resn;
	double m_max_dist_to_projn;
	Delaunay_cgal m_triangulation;
    };
}
