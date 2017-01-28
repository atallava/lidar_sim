#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/EllipsoidModelUtils.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class EllipsoidModelSim {
    public:
	EllipsoidModelSim();
	void setEllipsoidModels(EllipsoidModels ellipsoid_models);
	void setEllipsoidModels(std::string rel_path_models);
	void setLaserCalibParams(LaserCalibParams laser_calib_params);
	std::tuple<std::vector<std::vector<int> >,
	    std::vector<std::vector<double> > > calcEllipsoidIntersections(std::vector<double>, std::vector<std::vector<double> >);
	std::tuple<double, double> calcMahaDistRayToEllipsoid(std::vector<double> ray_origin, std::vector<double> ray_dirn, std::vector<double> mu, Eigen::MatrixXd cov_mat);
	
    private:
	double m_maxMahaDistForHit;
	EllipsoidModels m_ellipsoid_models;
	LaserCalibParams m_laser_calib_params;
    };
}
