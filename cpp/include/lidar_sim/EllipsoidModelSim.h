#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/MathUtils.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/LaserCalibParams.h>

namespace lidar_sim {
    class EllipsoidModelSim {
    public:
	EllipsoidModelSim();
	void loadEllipsoidModels(const std::string rel_path_models);
	void setEllipsoidModels(const EllipsoidModels &ellipsoid_models);
	void setLaserCalibParams(const LaserCalibParams laser_calib_params);
	void setDebugFlag(const int value);

	std::tuple<std::vector<std::vector<int> >,
	    std::vector<std::vector<double> > > calcEllipsoidIntersections(const std::vector<double>& ray_origin, const std::vector<std::vector<double> > &ray_dirns);

	std::tuple<std::vector<int>,
	    std::vector<double> > calcEllipsoidIntersections(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn);

	std::tuple<double, double> calcMahaDistRayToEllipsoid(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn, const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenIntersections(const std::vector<std::vector<int> > &intersection_flag, const std::vector<std::vector<double> > &dist_along_ray);

	std::vector<double> calcMahaDistPtToEllipsoids(const std::vector<int> &ellipsoid_ids, const std::vector<double> &pt);

	double calcMahaDistPtToEllipsoid(const int ellipsoid_id, const std::vector<double> &pt);

	double calcMahaDistPtToEllipsoid(const std::vector<double> &mu, const Eigen::MatrixXd &cov_mat, 
					 const std::vector<double> &pt);

	std::tuple<int, std::vector<int> >
	    assignEllipsoidHitCredits(const std::vector<double> &maha_dists_to_ellipsoids, 
				      const std::vector<int> &sorted_intersecting_ids,
				      const std::vector<double> &sorted_dist_alon_ray,
				      const double measured_range);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPose(const std::vector<double> &imu_pose);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenPoses(const std::vector<std::vector<double> > &imu_poses);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenRays(const std::vector<double> &ray_origin, const std::vector<std::vector<double> > &ray_dirns);
	std::tuple<std::vector<double>, int >
	    simPtGivenRay(const std::vector<double> &ray_origin, const std::vector<double> &ray_dirn);
	void setDeterministicSim(const bool choice);

	EllipsoidModels m_ellipsoid_models;
	LaserCalibParams m_laser_calib_params;

    private:
	double m_max_maha_dist_for_hit;
	int m_debug_flag;
	std::mt19937 m_gen;
	bool m_deterministic_sim;
    };
}
