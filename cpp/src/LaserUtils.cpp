#include <iostream>
#include <math.h>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>

namespace lidar_sim {
    std::vector<std::vector<double> > genRayDirnsWorldFrame(std::vector<double> imu_pose, LaserCalibParams laser_calib_params)
    {
	Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);
	Eigen::MatrixXd T_laser_world = laser_calib_params.extrinsics.T_laser_imu*T_imu_world;

	std::vector<std::vector<double> > ray_dirns_laser_frame = genRayDirnsLaserFrame(laser_calib_params.intrinsics);
	std::vector<std::vector<double> > ray_dirns_world_frame = rotateRayDirns(T_laser_world.block(0,0,3,3),
										 ray_dirns_laser_frame);

	return ray_dirns_world_frame;
    }

    std::vector<std::vector<double> > genRayDirnsLaserFrame(LaserIntrinsics intrinsics)
    {
	size_t n_alpha_vec = intrinsics.alpha_vec.size();
	size_t n_theta_vec = intrinsics.theta_vec.size();

	std::vector<std::vector<double> > ray_dirns;
	for(size_t i = 0; i < n_alpha_vec; ++i)
	{
	    double alpha = intrinsics.alpha_vec[i];
	    for(size_t j = 0; j < n_theta_vec; ++j)
	    {
		double theta = intrinsics.theta_vec[j];
		std::vector<double> ray_dirn = 
		    {std::cos(theta)*std::cos(alpha), std::sin(theta)*std::cos(alpha), std::sin(alpha)};
		ray_dirns.push_back(ray_dirn);
	    }
	}
	
	return ray_dirns;
    }

    std::vector<std::vector<double> > rotateRayDirns(Eigen::MatrixXd R, std::vector<std::vector<double> > ray_dirns)
    {
	Eigen::MatrixXd ray_dirns_eigen = stlArrayToEigen(ray_dirns);
	
	Eigen::MatrixXd ray_dirns_rotated_eigen = R*ray_dirns_eigen.transpose(); // now [3,n_rays]

	return EigenToStlArray(ray_dirns_rotated_eigen.transpose());
    }

    std::tuple<std::vector<double>, double> calcRayDirn(std::vector<double> ray_origin, std::vector<double> end_pt)
    {
	std::vector<double> ray_dirn(3,0);
	double meas_dist = 0;

	for(size_t i = 0; i < 3; ++i)
	{
	    ray_dirn[i] = end_pt[i] - ray_origin[i];
	    meas_dist += std::pow(ray_dirn[i], 2);
	}
	meas_dist = std::sqrt(meas_dist);
	for(size_t i = 0; i < 3; ++i)
	    ray_dirn[i] /= meas_dist;

	return std::make_tuple(ray_dirn, meas_dist);
    }
}
