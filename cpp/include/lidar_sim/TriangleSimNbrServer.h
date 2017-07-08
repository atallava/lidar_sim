#pragma once
#include <vector>
#include <string>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/TriangleModels.h>
#include <lidar_sim/FlannDatasetWrapper.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class TriangleSimNbrServer {
    public:
	TriangleSimNbrServer();
	TriangleSimNbrServer(const std::vector<TriangleModels> &triangle_models_vec);
	void setTriangleModels(const std::vector<TriangleModels> &triangle_models_vec);
	TriangleModelSim createSim(const std::vector<double> &pt);
	TriangleModelSim createSim(const std::vector<std::vector<double> > &pts);
	TriangleModelSim createSim(const std::vector<double> &ray_origin, 
				    const std::vector<double> &ray_dirn);
	TriangleModelSim createSim(const std::vector<double> &ray_origin, 
				    const std::vector<std::vector<double> > &ray_dirns);
	TriangleModelSim createSimGivenTriangleModels(const TriangleModels &triangle_models);
	std::vector<std::vector<double> > getNodesAlongRay(const std::vector<double> &ray_origin, 
							   const std::vector<double> &ray_dirn);
	std::vector<std::vector<double> > getNodesAlongRays(const std::vector<double> &ray_origin, 
							    const std::vector<std::vector<double> > &ray_dirns);

    private:
	double m_nn_radius;
	std::vector<TriangleModels> m_triangle_models_vec;
	TriangleModels m_triangle_models;
	FlannDatasetWrapper m_flann_helper;
	LaserCalibParams m_laser_calib_params;
    };
}
