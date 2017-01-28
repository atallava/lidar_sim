#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <lidar_sim/PoseUtils.h>
#include <lidar_sim/DataProcessingUtils.h>

namespace lidar_sim {
    Eigen::MatrixXd getImuTransfFromPose(std::vector<double> imu_pose)
    {
	double y = imu_pose[0];
	double x = imu_pose[1];
	double z = imu_pose[2];
	double roll = imu_pose[3];
	double pitch = imu_pose[4];
	double yaw = imu_pose[5];

	Eigen::MatrixXd T_pose(4,4);

	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d rotationMatrix = (yawAngle*pitchAngle*rollAngle).toRotationMatrix();

	for (size_t i = 0; i < 3; ++i)
	    for (size_t j = 0; j < 3; ++j)
		T_pose(i,j) = rotationMatrix(i,j);
	T_pose(0,3) = x; T_pose(1,3) = y; T_pose(2,3) = z;
	T_pose(3,0) = 0; T_pose(3,1) = 0; T_pose(3,2) = 0; T_pose(3,3) = 1;

	return T_pose;
    }

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
	size_t n_rays = n_alpha_vec*n_theta_vec;
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
}
