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

namespace lidar_sim {
    class GroundModeler {
    public:
	GroundModeler();
	void fitSmoothedSurface();
	void fitSmoothedPts();
	std::vector<std::vector<double> >
	    cutNodesToPtsProjn(std::vector<std::vector<double> > xy_nodes);
	std::tuple<std::vector<double>, std::vector<double> >
	    genNodeVecsForSmoothedFit(const std::vector<std::vector<double> > &pts);

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
    };
}
