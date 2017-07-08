#pragma once
#include <vector>
#include <string>

#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/FlannDatasetWrapper.h>
#include <lidar_sim/EllipsoidModelSim.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class EllipsoidSimNbrServer {
    public:
	EllipsoidSimNbrServer();
	EllipsoidSimNbrServer(const std::vector<EllipsoidModel> &ellipsoid_models);
	void setEllipsoidModels(const std::vector<EllipsoidModel> &ellipsoid_models);
	EllipsoidModelSim createSim(const std::vector<double> &pt);
	EllipsoidModelSim createSim(const std::vector<std::vector<double> > &pts);
	EllipsoidModelSim createSim(const std::vector<double> &ray_origin, 
				    const std::vector<double> &ray_dirn);
	EllipsoidModelSim createSim(const std::vector<double> &ray_origin, 
				    const std::vector<std::vector<double> > &ray_dirns);
	EllipsoidModelSim createSimGivenEllipsoids(const std::vector<EllipsoidModel> &ellipsoid_models);

    private:
	double m_nn_radius;
	std::vector<EllipsoidModel> m_ellipsoid_models;
	int m_n_ellipsoids;
	std::vector<std::vector<double> > m_ellipsoid_centers;
	FlannDatasetWrapper m_flann_helper;
	LaserCalibParams m_laser_calib_params;
    };
}
