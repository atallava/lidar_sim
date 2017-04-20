#include <iostream>
#include <math.h>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/PoseUtils.h>

namespace lidar_sim {
    std::vector<std::vector<double> > genRayDirnsWorldFrame(const std::vector<double> &imu_pose, const LaserCalibParams laser_calib_params)
    {
	Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);
	Eigen::MatrixXd T_laser_world = laser_calib_params.extrinsics.T_laser_imu*T_imu_world;

	std::vector<std::vector<double> > ray_dirns_laser_frame = genRayDirnsLaserFrame(laser_calib_params.intrinsics);
	std::vector<std::vector<double> > ray_dirns_world_frame = rotateRayDirns(T_laser_world.block(0,0,3,3),
										 ray_dirns_laser_frame);

	return ray_dirns_world_frame;
    }

    std::vector<std::vector<double> > genRayDirnsLaserFrame(const LaserIntrinsics intrinsics)
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

    std::vector<std::vector<double> > rotateRayDirns(const Eigen::MatrixXd &R, const std::vector<std::vector<double> > &ray_dirns)
    {
	Eigen::MatrixXd ray_dirns_eigen = stlArrayToEigen(ray_dirns);
	
	Eigen::MatrixXd ray_dirns_rotated_eigen = R*ray_dirns_eigen.transpose(); // now [3,n_rays]

	return EigenToStlArray(ray_dirns_rotated_eigen.transpose());
    }

    std::vector<double> laserPosnFromImuPose(const std::vector<double> &imu_pose, const LaserCalibParams laser_calib_params)
    {
	Eigen::MatrixXd T_imu_world = getImuTransfFromPose(imu_pose);
	Eigen::MatrixXd T_laser_world = T_imu_world*laser_calib_params.extrinsics.T_laser_imu;
	std::vector<double> laser_posn(3,0);
	for(size_t i = 0; i < 3; ++i)
	    laser_posn[i] = T_laser_world(i,3);

	return laser_posn;
    }
    
}
