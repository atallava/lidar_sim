#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <tuple>
#include <random>

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
	void setDebugFlag(int value);

	std::tuple<std::vector<std::vector<int> >,
	    std::vector<std::vector<double> > > calcEllipsoidIntersections(std::vector<double>, std::vector<std::vector<double> >);

	std::tuple<std::vector<int>,
	    std::vector<double> > calcEllipsoidIntersections(std::vector<double> ray_origin, std::vector<double> ray_dirn);

	std::tuple<double, double> calcMahaDistRayToEllipsoid(std::vector<double> ray_origin, std::vector<double> ray_dirn, std::vector<double> mu, Eigen::MatrixXd cov_mat);

	std::tuple<std::vector<std::vector<double> >, std::vector<int> >
	    simPtsGivenIntersections(std::vector<std::vector<int> > intersection_flag, std::vector<std::vector<double> > dist_along_ray);

	std::tuple<int, bool>
	    sampleHitId(std::vector<double> hit_prob_vec, std::vector<int> sorted_intersecting_ids);

	std::vector<double> calcMahaDistPtToEllipsoids(std::vector<int> ellipsoid_ids, std::vector<double> pt);

	double calcMahaDistPtToEllipsoid(int ellipsoid_id, std::vector<double> pt);

	double calcMahaDistPtToEllipsoid(std::vector<double> mu, Eigen::MatrixXd cov_mat, 
							std::vector<double> pt);

	std::tuple<int, std::vector<int> >
	    assignEllipsoidHitCredits(std::vector<double> maha_dists_to_ellipsoids, 
							 std::vector<int> sorted_intersecting_ids);

	std::vector<std::vector<double> > simPtsGivenPose(std::vector<double> imu_pose);

	std::vector<std::vector<double> > simPtsGivenPoses(std::vector<std::vector<double> > imu_poses);

    private:
	double m_max_maha_dist_for_hit;
	int m_debug_flag;
	EllipsoidModels m_ellipsoid_models;
	LaserCalibParams m_laser_calib_params;
	std::mt19937 m_gen;
    };
}
